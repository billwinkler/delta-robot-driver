(ns delta-robot.command-driver
  (:require [babashka.process :refer [sh]]
            [clojure.string :as str]
            [clojure.tools.logging :as log]
            [delta-robot.config :refer [motor-step-pins motor-direction-pins limit-switch-pins]]
            [delta-robot.pulse-timing :as timing]))

;; --- Utility Functions ---

(defn- execute-pigs-cmd
  "Executes a pigpio command via the 'pigs' utility and returns trimmed output."
  [& args]
  (if (> (count args) 15)
    (log/info "cmd: pigs" (str/join " " (take 15 args)) "...")
    (log/info "cmd: pigs" (str/join " " args)))
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

(defn- clear-waveforms
  "Clears all existing pigpio waveforms."
  []
  (execute-pigs-cmd "wvclr"))

(defn- set-direction-pins
  "Sets direction pins for all motors based on their command map."
  [commands]
  (log/info "setting direction pins:" commands)
  (let [dir-pins (motor-direction-pins)]
    (doseq [[motor-id {:keys [direction]}] commands]
      ;; Safely get the direction pin from the vector using the motor-id as the index.
      (when-let [dir-pin (nth dir-pins motor-id nil)]
        (execute-pigs-cmd "w" (str dir-pin) (str direction))))))

;; --- Limit Switch Monitoring ---
(defn- monitor-limit-switches
  "Monitors limit switches in a separate thread."
  [wave-ids wave-pid]
  (future
    (while (= "1" (execute-pigs-cmd "wvbsy"))
      (when (check-limit-switches wave-ids wave-pid)
        (Thread/sleep 5)))
    (doseq [wid wave-ids]
      (execute-pigs-cmd "wvdel" (str wid)))
    (log/info "Waveform finished or stopped.")))

(defn- is-motor-at-limit?
  "Checks if a motor is being commanded up while its limit switch is pressed."
  [motor-id direction]
  (if (= direction 1)
    (let [limit-pin (nth (limit-switch-pins) motor-id)
          limit-value (execute-pigs-cmd "r" (str limit-pin))]
      (= "0" limit-value))
    false))

;; --- Waveform Execution ---
(defn- start-waveform-chain
  "Starts a waveform chain process for the given wave-ids and loop count, and returns its PID."
  [wave-ids loop-count]
  (let [repeats (max 0 (- loop-count 1))
        lo (mod repeats 256)
        hi (quot repeats 256)
        chain (concat (when (> loop-count 1) ["255" "1" (str lo) (str hi)])
                      (map str wave-ids)
                      (when (> loop-count 1) ["255" "2"]))
        command-args (into ["wvcha"] chain)
        cmd-str (str/join " " (cons "pigs" command-args))
        process (sh "bash" "-c" (str cmd-str " & echo $!"))]
    (if (zero? (:exit process))
      (let [pid (str/trim (:out process))]
        (log/infof "wvcha process started with PID: %s for command: %s" pid cmd-str)
        pid)
      (do
        (log/errorf "Failed to start wvcha: %s" (:err process))
        nil))))

(defn- create-and-run-wave
  "Adds waveforms to pigpiod, creates them, and starts the chain."
  [waveforms loop-count]
  (when (pos? loop-count)
    (clear-waveforms)
    (let [wave-ids (keep (fn [pulse-chunk]
                           (apply execute-pigs-cmd "wvag" (flatten pulse-chunk))
                           (try
                             (Integer/parseInt (execute-pigs-cmd "wvcre"))
                             (catch Exception e
                               (log/errorf "Failed to parse wave-id: %s" (.getMessage e))
                               nil)))
                         waveforms)]
      (if (and (seq wave-ids) (every? #(>= % 0) wave-ids))
        (do
          (log/info "Waveforms created with IDs:" wave-ids)
          (if-let [wave-pid (start-waveform-chain wave-ids loop-count)]
            (do
              (monitor-limit-switches wave-ids wave-pid)
              (log/infof "Waveform chain started for wave-ids %s" wave-ids))
            (log/error "Failed to start waveform chain")))
        (log/error "Failed to create one or more waveforms")))))

;; --- Main Public Function ---

(defn send-commands
  "Processes motor commands, generates a synchronized waveform, and executes it."
  [commands]
  (log/infof "Processing motor commands: %s" commands)
  ;; Prevent motors from moving up if they are already at the limit switch.
  (let [checked-commands (into {}
                               (map (fn [[motor-number {:keys [total-pulses direction] :as command}]]
                                      (log/info "motor:" motor-number)
                                      (if (is-motor-at-limit? motor-number direction)
                                        (do
                                          (log/warnf "Motor %d is at its limit and commanded to move up. Ignoring command." motor-number)
                                          [motor-number (assoc command :total-pulses 0)])
                                        [motor-number command]))
                                    commands))
        ;; Ensure commands are sorted by motor-id to maintain consistent pin order
        sorted-commands (sort-by key checked-commands)
        step-counts (map (comp :total-pulses val) sorted-commands)
        step-pins (motor-step-pins)]

    (log/info "Checked-commands:" checked-commands)
    (log/info "Setting motor directions...")
    (set-direction-pins checked-commands)

    (log/info "Generating synchronized waveform for steps:" step-counts "on pins:" step-pins)
    (let [{:keys [waveforms loop-count]} (timing/generate-waveform-chain step-counts step-pins)]
      (do (log/info "loop-count:" loop-count "waveforms:" (count waveforms))
        (if (and (seq waveforms) (pos? loop-count))
          (create-and-run-wave waveforms loop-count)
          (log/info "No movement required (zero pulses or empty waveform)."))))))

;; --- Homing ---

(defn home-motors
  "Executes the homing sequence for all motors in parallel.
  Each motor moves up until its limit switch is triggered."
  []
  (log/info "Starting homing sequence for all motors.")
  (let [motor-ids (range (count (motor-step-pins)))
        step-pins (motor-step-pins)
        dir-pins (motor-direction-pins)
        limit-pins (limit-switch-pins)
        homing-freq 1600
        homing-direction 1
        ;; Atomically track which motors still need to be homed.
        motors-to-home (atom (into #{}
                                   (filter (fn [id]
                                             (let [limit-pin (nth limit-pins id)]
                                               (not= "0" (execute-pigs-cmd "r" (str limit-pin))))))
                                   motor-ids))]

    (if (empty? @motors-to-home)
      (log/info "All motors are already home.")
      (do
        (log/infof "Motors to be homed: %s" @motors-to-home)

        ;; Start all non-homed motors.
        (doseq [motor-id @motors-to-home]
          (let [dir-pin (nth dir-pins motor-id)
                step-pin (nth step-pins motor-id)]
            (execute-pigs-cmd "w" (str dir-pin) (str homing-direction))
            (execute-pigs-cmd "pfs" (str step-pin) (str homing-freq))
            (execute-pigs-cmd "p" (str step-pin) "128")))

        ;; Poll limit switches and stop motors individually.
        (while (not-empty @motors-to-home)
          (doseq [motor-id @motors-to-home]
            (let [limit-pin (nth limit-pins motor-id)]
              (when (= "0" (execute-pigs-cmd "r" (str limit-pin)))
                (log/infof "Motor %d reached home." motor-id)
                (let [step-pin (nth step-pins motor-id)]
                  (execute-pigs-cmd "p" (str step-pin) "0"))
                (swap! motors-to-home disj motor-id))))
          (Thread/sleep 10))

        (log/info "Homing sequence complete.")))))

(comment
  ;; Example of moving three motors with different step counts.
  ;; This is now possible with the refactored driver.
  (let [commands {0 {:total-pulses 500, :direction 1} 
                  1 {:total-pulses 500, :direction 1} 
                  2 {:total-pulses 500, :direction 1}}]
    (send-commands commands))

  (let [commands {0 {:total-pulses 500, :direction 0} 
                  1 {:total-pulses 500, :direction 0} 
                  2 {:total-pulses 500, :direction 0}}]
    (send-commands commands))

  (home-motors)

  (timing/run-demo [100 101 102])
  (timing/run-demo [2000 1900 1800])
  )
