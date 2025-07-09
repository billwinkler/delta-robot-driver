```clojure
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
  "Executes a pigpio command and returns trimmed output. Throws an exception on failure."
  [& args]
  (log/info "cmd: pigs" (str/join " " args))
  (try
    (let [{:keys [out err exit]} (apply sh "pigs" args)]
      (if (zero? exit)
        (let [result (str/trim out)]
          (log/info "pigs result:" result)
          result)
        (do
          (log/errorf "Failed to execute pigs command %s: exit code %d, error: %s" args exit err)
          (throw (ex-info (str "pigpio command failed: " err) {:command args :exit exit :error err})))))
    (catch Exception e
      (log/errorf "Exception executing pigs command %s: %s" args (.getMessage e))
      (throw e))))

(defn- generate-waveform-data
  "Generates waveform data for a motor."
  [motor-id {:keys [total-pulses direction]}]
  (let [motor-keyword (keyword (str "motor" motor-id))
        gpio-step (get-in config [:gpio-pins motor-keyword :step])
        gpio-dir (get-in config [:gpio-pins motor-keyword :dir])]
    {:motor-id motor-id
     :gpio-step gpio-step
     :gpio-dir gpio-dir
     :direction direction
     :total-pulses total-pulses}))

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

(defn- start-waveform-process
  "Starts a waveform chain process for a fixed number of pulses. Returns its PID."
  [wave-id total-pulses]
  (when-not (pos? total-pulses)
    (throw (IllegalArgumentException. "total-pulses must be positive")))
  (let [x (mod total-pulses 256)
        y (quot total-pulses 256)
        command (format "wvcha 255 0 %d 255 1 %d %d" wave-id x y)
        full-command (str "pigs " command)]
    (log/info (format "delay: x: %s, y: %s, v: %s" x y (+ x (* y 256))))
    (log/info "cmd:" full-command)
    (try
      (let [{:keys [out err exit]} (sh "pigs" command)]
        (if (zero? exit)
          (let [process (check (sh "bash" "-c" (str full-command " & echo $!")))
                pid (str/trim (:out process))]
            (log/infof "wvcha process started with PID: %s" pid)
            pid)
          (do
            (log/errorf "Failed to execute pigs wvcha command: exit code %d, error: %s" exit err)
            (throw (ex-info (str "pigpio wvcha command failed: " err)
                            {:command full-command :exit exit :error err})))))
      (catch Exception e
        (log/errorf "Exception executing pigs wvcha command %s: %s" full-command (.getMessage e))
        (throw e)))))

(defn- create-bitmask
  "Creates a bitmask from a list of GPIO pins."
  [pins]
  (reduce bit-or 0 (map #(bit-shift-left 1 %) pins)))

(defn generate-waveforms
  "Generates and starts waveforms for step pins with a fixed number of pulses."
  [step-pins total-pulses]
  (validate-config)
  (let [mask (create-bitmask step-pins)]
    (clear-waveforms)
    (execute-pigs-cmd "wvag" mask 0 500 0 mask 500)
    (if-let [wave-id (try (Integer/parseInt (execute-pigs-cmd "wvcre"))
                         (catch Exception e
                           (log/errorf "Failed to parse wave-id: %s" (.getMessage e))
                           nil))]
      (if (>= wave-id 0)
        (do
          (log/info "wave-id:" wave-id)
          (let [wave-pid (start-waveform-process wave-id total-pulses)]
            (monitor-limit-switches wave-id wave-pid)
            (log/infof "Waveform started with ID %s" wave-id)))
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
        total-pulses (:total-pulses (first wave-data-per-motor))]
    (when-not (every? #(= (:total-pulses %) total-pulses) wave-data-per-motor)
      (throw (IllegalArgumentException. "All motors must have the same total-pulses for this test")))
    (log/info "step-pins:" step-pins)
    (log/info "wave-data count:" (count wave-data-per-motor))
    (clear-waveforms)
    (set-direction-pins wave-data-per-motor)
    (generate-waveforms step-pins total-pulses)))

(comment
  (let [commands {0 {:total-pulses 1000, :direction 1}
                  1 {:total-pulses 1000, :direction 1}
                  2 {:total-pulses 1000, :direction 1}}]
    (send-commands commands))
  )
