(ns delta-robot.command-driver
  (:require [babashka.process :refer [sh]]
            [clojure.java.io :as io]
            [clojure.string :as str]
            [delta-robot.config :refer [config]]))

(def min-phase-pulses 10)
(def time-scale-precision 10000) ; A constant (set to 10000) that acts as a multiplier to maintain precision during floating-point time calculations, especially when scaling pulse periods to synchronize motor movements.
(def pulse-on-duration-us 10) ; Duration for step pin to be HIGH in microseconds
(def limit-switch-pins (get-in config [:gpio-pins :limit-switches]))

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

(defn- monitor-limit-switches [wave-id wave-pid]
  (let [limit-pins limit-switch-pins]
    (future
      (loop []
        (let [pin-values (map #(-> (sh "pigs" "r" (str %)) :out str/trim) limit-pins)]
          (when (some #(= "0" %) pin-values)
            (println "Limit switch triggered! Stopping waveform.")
            (sh "pigs" "wvtx" "0")
            (when-not (str/blank? wave-pid)
              (sh "kill" wave-pid "2>/dev/null"))
            (sh "pigs" "wvdel" wave-id)
            (println "Waveform" wave-id "stopped and deleted.")))
        (Thread/sleep 5)
        (if (= "1" (str/trim (:out (sh "pigs" "wvbsy"))))
          (recur)
          (println "Waveform finished or stopped."))))))

(defn- start-wvcha-process [wave-id pulses]
  (let [x (mod pulses 256)
        y (quot pulses 256)
        command (format "pigs wvcha 255 0 %d 255 1 %d %d 255 2 0" wave-id x y)
        process (sh "bash" "-c" (str command " & echo $!"))
        pid (str/trim (:out process))]
    (println "wvcha process started with PID:" pid)
    pid))

(defn generate-waveforms [step-pins pulses]
  (let [mask (reduce bit-or 0 (map #(bit-shift-left 1 %) step-pins))]
    (sh "pigs" "wvclr")
    (sh "pigs" "wvag" mask 0 500 0 mask 500)
    (let [wave-id-str (str/trim (:out (sh "pigs" "wvcre")))]
      (if-let [wave-id (try (Integer/parseInt wave-id-str) (catch Exception _ nil))]
        (if (>= wave-id 0)
          (let [wave-pid (start-wvcha-process wave-id pulses)]
            (monitor-limit-switches wave-id wave-pid)
            (println "Waveform started with ID" wave-id ". Monitoring limit switches..."))
          (println "Error: Failed to create waveform, received wave-id:" wave-id))
        (println "Error: Failed to parse wave-id:" wave-id-str)))))

(defn send-commands [commands]
  (println "Sending commands:" commands)
  (let [wave-data-per-motor (doall (map (fn [[motor-id cmd]]
                                          (generate-waveform-data motor-id cmd))
                                        (vec commands)))
        step-pins (map :gpio-step wave-data-per-motor)
        ;; For wvcha, we can use a large number for continuous motion or a specific number.
        ;; Using 0 for pulses in wvcha means loop forever.
        pulses 0]
    (clear-and-set-directions wave-data-per-motor)
    (generate-waveforms step-pins pulses)))

(comment
  (let [commands {0 {:total-pulses 1000, :direction 1}
                  1 {:total-pulses 1000, :direction 1}
                  2 {:total-pulses 1000, :direction 1}}]
    (send-commands commands))


  )
