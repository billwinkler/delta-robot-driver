(ns delta-robot.command-driver
  (:require [babashka.process :refer [sh check]]
            [clojure.java.io :as io]
            [clojure.string :as str]
            [clojure.tools.logging :as log]
            [delta-robot.config :refer [config]]))

;; Constants
(def ^:const min-phase-pulses 10)
(def ^:const time-scale-precision 10000)
(def ^:const pulse-on-duration-us 10)
(def ^:const limit-switch-pins (get-in config [:gpio-pins :limit-switches]))

;; Utility Functions
(defn- validate-config
  "Validates required configuration keys."
  []
  (when-not (and (:max-frequency config)
                 (:min-frequency config)
                 (:pulse-overhead-ns config)
                 (:acceleration-pulses config)
                 (:deceleration-pulses config)
                 (:gpio-pins config))
    (throw (IllegalStateException. "Missing required config keys"))))

(defn- execute-pigs-cmd
  "Executes a pigpio command and returns trimmed output."
  [& args]
  (try
    (-> (apply sh "pigs" args) :out str/trim)
    (catch Exception e
      (log/errorf "Failed to execute pigs command %s: %s" args (.getMessage e))
      (throw e))))

(defn- calculate-period
  "Calculates period for a pulse based on position in accel/decel phases."
  [k total-pulses accel-pulses decel-pulses min-period max-period]
  (cond
    (and (< k accel-pulses) (> accel-pulses 1))
    (- max-period (* (- max-period min-period) (/ k (dec accel-pulses))))
    (and (>= k (- total-pulses decel-pulses)) (> decel-pulses 1))
    (let [m (- total-pulses 1 k)]
      (+ min-period (* (- max-period min-period) (/ (- decel-pulses 1 m) (dec decel-pulses)))))
    :else min-period))

(defn- estimate-motion-duration
  "Estimates total motion duration in nanoseconds."
  [total-pulses accel-pulses decel-pulses]
  (let [min-period (long (/ 1e9 (:max-frequency config)))
        max-period (long (/ 1e9 (:min-frequency config)))
        pulse-overhead (:pulse-overhead-ns config)
        constant-pulses (max 0 (- total-pulses accel-pulses decel-pulses))]
    (if (<= total-pulses 1)
      0
      (long
        (+ (* (+ max-period min-period) accel-pulses 1/2)
           (* accel-pulses pulse-overhead)
           (* constant-pulses (+ min-period pulse-overhead))
           (* (+ min-period max-period) decel-pulses 1/2)
           (* decel-pulses pulse-overhead))))))

(defn- generate-waveform-data
  "Generates waveform data for a motor."
  [motor-id {:keys [total-pulses direction]}]
  (let [motor-keyword (keyword (str "motor" motor-id))
        gpio-step (get-in config [:gpio-pins motor-keyword :step])
        gpio-dir (get-in config [:gpio-pins motor-keyword :dir])
        [accel-pulses decel-pulses] (if (> total-pulses (+ (:acceleration-pulses config) (:deceleration-pulses config)))
                                      [(:acceleration-pulses config) (:deceleration-pulses config)]
                                      (let [half-pulses (quot total-pulses 2)]
                                        [half-pulses (- total-pulses half-pulses)]))
        duration (estimate-motion-duration total-pulses accel-pulses decel-pulses)
        min-period (/ 1e9 (:max-frequency config))
        max-period (/ 1e9 (:min-frequency config))
        periods-ns (mapv #(calculate-period % total-pulses accel-pulses decel-pulses min-period max-period)
                        (range total-pulses))]
    {:motor-id motor-id
     :gpio-step gpio-step
     :gpio-dir gpio-dir
     :direction direction
     :duration duration
     :periods-ns periods-ns}))

(defn- clear-waveforms
  "Clears all existing pigpio waveforms."
  []
  (execute-pigs-cmd "wvclr"))

(defn- set-direction-pins
  "Sets direction pins for all motors."
  [wave-data-per-motor]
  (doseq [{:keys [gpio-dir direction]} wave-data-per-motor]
    (execute-pigs-cmd "w" (str gpio-dir) (str direction))))

(defn- check-limit-switches
  "Checks limit switches and stops waveform if triggered. Returns true if waveform should continue."
  [wave-id wave-pid]
  (try
    (let [pin-values (mapv #(execute-pigs-cmd "r" (str %)) limit-switch-pins)]
      (if (some #(= "0" %) pin-values)
        (do
          (log/warn "Limit switch triggered! Stopping waveform.")
          (execute-pigs-cmd "wvtx" "0")
          (when-not (str/blank? wave-pid)
            ;; kill with errors suppressed in the event the process is already gone
            (sh "kill" wave-pid "2>/dev/null"))
          (execute-pigs-cmd "wvdel" wave-id)
          (log/infof "Waveform %s stopped and deleted." wave-id)
          false)
        true))
    (catch Exception e
      (log/errorf "Error in limit switch monitoring: %s" (.getMessage e))
      true)))

(defn- monitor-limit-switches
  "Monitors limit switches and stops waveform if triggered."
  [wave-id wave-pid]
  (future
    (while (= "1" (execute-pigs-cmd "wvbsy"))
      (when (check-limit-switches wave-id wave-pid)
        (Thread/sleep 5)))
    (log/info "Waveform finished or stopped.")))

(defn- start-wvcha-process
  "Starts a waveform chain process."
  [wave-id pulses]
  (let [x (mod pulses 256)
        y (quot pulses 256)
        command (format "pigs wvcha 255 0 %d 255 1 %d %d 255 2 0" wave-id x y)
        process (check (sh "bash" "-c" (str command " & echo $!")))
        pid (str/trim (:out process))]
    (log/infof "wvcha process started with PID: %s" pid)
    pid))

(defn- create-bitmask
  "Creates a bitmask from a list of GPIO pins."
  [pins]
  (reduce bit-or 0 (map #(bit-shift-left 1 %) pins)))

(defn generate-waveforms
  "Generates and starts waveforms for step pins."
  [step-pins pulses]
  (validate-config)
  (let [mask (create-bitmask step-pins)]
    (clear-waveforms)
    (execute-pigs-cmd "wvag" mask 0 500 0 mask 500)
    (if-let [wave-id (try (Integer/parseInt (execute-pigs-cmd "wvcre"))
                         (catch Exception e
                           (log/errorf "Failed to parse wave-id: %s" (.getMessage e))
                           nil))]
      (if (>= wave-id 0)
        (let [wave-pid (start-wvcha-process wave-id pulses)]
          (monitor-limit-switches wave-id wave-pid)
          (log/infof "Waveform started with ID %s" wave-id))
        (log/errorf "Failed to create waveform, received wave-id: %s" wave-id))
      (log/error "Failed to create waveform"))))

(defn send-commands
  "Processes motor commands and generates waveforms."
  [commands]
  (log/infof "Sending commands: %s" commands)
  (validate-config)
  (let [wave-data-per-motor (mapv (fn [[motor-id cmd]] (generate-waveform-data motor-id cmd))
                                 commands)
        step-pins (mapv :gpio-step wave-data-per-motor)
        pulses 0]
    (clear-waveforms)
    (set-direction-pins wave-data-per-motor)
    (generate-waveforms step-pins pulses)))

(comment
  (let [commands {0 {:total-pulses 1000, :direction 1}
                  1 {:total-pulses 1000, :direction 1}
                  2 {:total-pulses 1000, :direction 1}}]
    (send-commands commands))
  )
