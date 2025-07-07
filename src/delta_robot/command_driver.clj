(ns delta-robot.command-driver
  (:require [babashka.process :refer [sh]]
            [clojure.java.io :as io]
            [clojure.string :as str]
            [delta-robot.config :refer [config]]))

(def min-phase-pulses 10)
(def time-scale-precision 10000) ; A constant (set to 10000) that acts as a multiplier to maintain precision during floating-point time calculations, especially when scaling pulse periods to synchronize motor movements.
(def pulse-on-duration-us 10) ; Duration for step pin to be HIGH in microseconds

(defn- estimate-motion-duration [total-pulses accel-pulses decel-pulses]
  (let [min-period (long (/ 1e9 (:max-frequency config)))
        max-period (long (/ 1e9 (:min-frequency config)))
        pulse-overhead (:pulse-overhead-ns config)
        constant-pulses (if (> total-pulses (+ accel-pulses decel-pulses))
                          (- total-pulses accel-pulses decel-pulses)
                          0)]
    (if (<= total-pulses 1)
      0
      (long
        (+ (* (+ max-period min-period) accel-pulses 1/2)
           (* accel-pulses pulse-overhead)
           (* constant-pulses (+ min-period pulse-overhead))
           (* (+ min-period max-period) decel-pulses 1/2)
           (* decel-pulses pulse-overhead))))))

(defn- generate-waveform-data [motor-id {:keys [total-pulses direction]}]
  (let [motor-keyword (keyword (str "motor" motor-id))
        gpio-step (get-in config [:gpio-pins motor-keyword :step])
        gpio-dir (get-in config [:gpio-pins motor-keyword :dir])
        accel-pulses-cfg (:acceleration-pulses config)
        decel-pulses-cfg (:deceleration-pulses config)
        [accel-pulses decel-pulses]
        (if (> total-pulses (+ accel-pulses-cfg decel-pulses-cfg))
          [accel-pulses-cfg decel-pulses-cfg]
          (let [half-pulses (quot total-pulses 2)]
            [half-pulses (- total-pulses half-pulses)]))

        duration (estimate-motion-duration total-pulses accel-pulses decel-pulses)

        periods-ns
        (loop [k 0
               acc []]
          (if (>= k total-pulses)
            acc
            (let [min-period (/ 1e9 (:max-frequency config))
                  max-period (/ 1e9 (:min-frequency config))
                  period-accel (if (and (< k accel-pulses) (> accel-pulses 1))
                                 (- max-period (* (- max-period min-period) (/ k (dec accel-pulses))))
                                 min-period)
                  period-decel (if (and (>= k (- total-pulses decel-pulses)) (> decel-pulses 1))
                                 (let [m (- total-pulses 1 k)]
                                   (+ min-period (* (- max-period min-period) (/ (- decel-pulses 1 m) (dec decel-pulses)))))
                                 min-period)
                  period-k (max period-accel period-decel)]
              (recur (inc k) (conj acc period-k)))))] ; Store period in ns
    {:motor-id motor-id
     :gpio-step gpio-step
     :gpio-dir gpio-dir
     :direction direction
     :duration duration ; total duration of this motor's movement
     :periods-ns periods-ns})) ; List of period (delay) for each step in ns

(defn- clear-and-set-directions [wave-data-per-motor]
  "Clears existing pigpio waveforms and sets the direction pins for all motors."
  (sh "pigs" "wvclr")
  (doseq [wd wave-data-per-motor]
    (sh "pigs" "w" (str (:gpio-dir wd)) (str (:direction wd)))))

(defn- populate-gpio-event-queue [wave-data-per-motor max-duration]
  "Populates an event queue with all pulse ON/OFF transitions across all motors,
  synchronized to the max-duration."
  (let [event-queue (atom (sorted-map))] ; Map: timestamp_us -> {:on-gpios #{}, :off-gpios #{}}
    (doseq [wd wave-data-per-motor]
      (let [gpio-step (:gpio-step wd)
            time-scale (if (pos? (:duration wd))
                         (/ (* max-duration (double time-scale-precision)) (:duration wd))
                         time-scale-precision)
            current-time-us (atom 0)]
        (doseq [period-ns (:periods-ns wd)]
          (let [scaled-period-us (long (/ (* period-ns time-scale) time-scale-precision 1000)) ; Convert to us
                ;; Ensure scaled_period_us is at least pulse-on-duration-us + 1 (for minimal OFF time)
                min-valid-period-us (+ pulse-on-duration-us 1)
                adjusted-scaled-period-us (max scaled-period-us min-valid-period-us)]
            ;; Add event to turn step pin ON
            (swap! event-queue update-in [@current-time-us :on-gpios] (fnil conj #{}) gpio-step)
            ;; Add event to turn step pin OFF after pulse-on-duration-us
            (swap! event-queue update-in [@current-time-us (+ @current-time-us pulse-on-duration-us) :off-gpios] (fnil conj #{}) gpio-step)
            ;; Advance time for the start of the next pulse (total period)
            (swap! current-time-us + adjusted-scaled-period-us))))) ; Use adjusted period here
    @event-queue))

(defn- generate-wvag-segments [event-queue wave-data-per-motor]
  "Converts the event queue into a sequence of (on_mask, off_mask, delay) triplets
  suitable for `pigs wvag`."
  (let [final-wvag-segments (atom [])
        current-time-us (atom 0)
        current-gpio-levels (atom {}) ; Map: gpio -> 0/1 (tracks the current state of all relevant GPIOs)
        all-gpios (set (map :gpio-step wave-data-per-motor))]

    ;; Initialize all relevant GPIOs to LOW (0)
    (doseq [gpio all-gpios]
      (swap! current-gpio-levels assoc gpio 0))

    (doseq [[event-time events] event-queue]
      (let [delay-since-last-event (- event-time @current-time-us)]
        ;; If there's a time gap before this event, add a delay segment
        (when (> delay-since-last-event 0)
          (swap! final-wvag-segments conj (str "0 0 " delay-since-last-event)))

        ;; Update current GPIO levels based on the events at this timestamp
        (let [on-gpios (get events :on-gpios #{})
              off-gpios (get events :off-gpios #{})]
          (doseq [gpio on-gpios] (swap! current-gpio-levels assoc gpio 1))
          (doseq [gpio off-gpios] (swap! current-gpio-levels assoc gpio 0)))

        ;; Construct the on_mask and off_mask based on the *new* current state of all GPIOs
        (let [effective-on-mask (reduce bit-or 0 (map (fn [gpio] (bit-shift-left 1 gpio))
                                                       (filter (fn [gpio] (= 1 (get @current-gpio-levels gpio))) all-gpios)))
              effective-off-mask (reduce bit-or 0 (map (fn [gpio] (bit-shift-left 1 gpio))
                                                        (filter (fn [gpio] (= 0 (get @current-gpio-levels gpio))) all-gpios)))]
          ;; Add the state change segment with 0 delay (the actual delay was handled by the previous segment)
          (swap! final-wvag-segments conj (str effective-on-mask " " effective-off-mask " 0")))

        ;; Advance the current time to the time of the current event
        (reset! current-time-us event-time)))

    ;; Filters out any `0 0 0` segments. These segments would represent a zero-delay, no-change operation,
    ;; which `pigpio` does not require and can sometimes lead to issues if not explicitly handled or removed.
    (filter (fn [s] (not= "0 0 0" s)) @final-wvag-segments)))

(defn- execute-waveform [wvag-segments]
  "Executes the pigpio waveform commands (wvag, wvcre, wvtx) and handles errors."
  (if (empty? wvag-segments)
    (println "No waveform segments generated. Nothing to send.")
    (let [wvag-command-args (mapcat (fn [s] (str/split s #"\s+")) wvag-segments)
          batch-size 150 ; Number of arguments per pigs wvag call (50 segments * 3 args/segment)
          batches (partition-all batch-size wvag-command-args)]

      ;; --- DEBUGGING START ---
      (println (str "DEBUG: Total number of wvag segments: " (count wvag-segments)))
      (println (str "DEBUG: Total pigs wvag arguments: " (count wvag-command-args)))
      (println (str "DEBUG: Number of wvag batches: " (count batches)))
      ;; (println "DEBUG: wvag command args (first 30):" (take 30 wvag-command-args)) ; Print first few args
      ;; (println "DEBUG: wvag command args (last 30):" (take-last 30 wvag-command-args)) ; Print last few args
      ;; (println "DEBUG: wvag segments:" wvag-segments) ; Uncomment to see all segments if needed
      ;; --- DEBUGGING END ---

      (doseq [batch batches]
        (let [result (apply sh (into ["pigs" "wvag"] batch))]
          (when-not (zero? (:exit result))
            (println "ERROR: pigs wvag batch failed:")
            (println "STDOUT:" (:out result))
            (println "STDERR:" (:err result))
            (throw (Exception. (str "Failed to add waveform segments. pigpio error: " (:err result)))))))

      (let [wvcre-result (sh "pigs" "wvcre") ; Capture the full result
            wave-id (-> wvcre-result :out str/trim Integer/parseInt)]
        ;; --- DEBUGGING START ---
        (println "DEBUG: pigs wvcre raw output:" (:out wvcre-result))
        (println "DEBUG: pigs wvcre stderr:" (:err wvcre-result))
        (println "DEBUG: wave-id parsed:" wave-id)
        ;; --- DEBUGGING END ---
        (if (neg? wave-id)
          (throw (Exception. (str "Failed to create waveform. pigpio error: " (:err wvcre-result)))) ; Include pigpio error in exception
          (let [result (sh "pigs" "wvtx" (str wave-id))]
            (if (zero? (:exit result))
              (println "Command sent successfully!")
              (do
                (println "Failed to send command:" (:err result))
                (println "STDOUT:" (:out result))))))))))

(defn send-commands [commands]
  (println "commands:" commands)
  (let [wave-data-per-motor (map (fn [cmd] (generate-waveform-data (:motor-number cmd) cmd)) commands)
        max-duration (apply max (map :duration wave-data-per-motor))
        event-queue (populate-gpio-event-queue wave-data-per-motor max-duration)
        wvag-segments (generate-wvag-segments event-queue wave-data-per-motor)]

    (clear-and-set-directions wave-data-per-motor)
    (execute-waveform wvag-segments)))
  
(defn send-debug-pulses [motor-id direction]
  "Sends 100 pulses to the specified motor in the given direction for debugging."
  (println (str "Sending pulses to motor " motor-id " in direction " direction))
  (send-commands [{:motor-number motor-id
                   :total-pulses 100
                   :direction direction}]))
