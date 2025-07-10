(ns delta-robot.command-driver
  (:require [babashka.process :refer [sh]]
            [clojure.string :as str]
            [clojure.tools.logging :as log]
            [delta-robot.config :refer [motor-step-pins motor-direction-pins limit-switch-pins]]
            [delta-robot.timing :as timing]))

;; --- Utility Functions ---

(defn- execute-pigs-cmd
  "Executes a pigpio command via the 'pigs' utility and returns trimmed output."
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

(defn- clear-waveforms
  "Clears all existing pigpio waveforms."
  []
  (execute-pigs-cmd "wvclr"))

(defn- set-direction-pins
  "Sets direction pins for all motors based on their command map."
  [commands]
  (let [dir-pins (motor-direction-pins)]
    (doseq [[motor-id {:keys [direction]}] commands]
      ;; Safely get the direction pin from the vector using the motor-id as the index.
      (when-let [dir-pin (nth dir-pins motor-id nil)]
        (execute-pigs-cmd "w" (str dir-pin) (str direction))))))

;; --- Limit Switch Monitoring ---

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
  "Monitors limit switches in a separate thread."
  [wave-id wave-pid]
  (future
    (while (= "1" (execute-pigs-cmd "wvbsy"))
      (when (check-limit-switches wave-id wave-pid)
        (Thread/sleep 5)))
    (log/info "Waveform finished or stopped.")))

;; --- Waveform Execution ---

(defn- start-waveform-chain
  "Starts a waveform chain process for a given loop count and returns its PID."
  [wave-id loop-count]
  (let [x (mod loop-count 256)
        y (quot loop-count 256)
        ;; This command sequence creates a loop that repeats the wave-id 'loop-count' times.
        command-args ["wvcha" "255" "0" (str wave-id) "255" "1" (str x) (str y)]
        process (sh "bash" "-c" (str (str/join " " (cons "pigs" command-args)) " & echo $!"))
        pid (str/trim (:out process))]
    (log/infof "wvcha process started with PID: %s" pid)
    pid))

(defn- create-and-run-wave
  "Adds a waveform to pigpiod, creates it, and starts the chain."
  [waveform loop-count]
  (when (pos? loop-count)
    (clear-waveforms)
    ;; The waveform is a list of lists, e.g., [[on off delay]...]. Flatten it for the command line.
    (apply execute-pigs-cmd "wvag" (flatten waveform))
    (if-let [wave-id (try (Integer/parseInt (execute-pigs-cmd "wvcre"))
                          (catch Exception e
                            (log/errorf "Failed to parse wave-id: %s" (.getMessage e))
                            nil))]
      (if (>= wave-id 0)
        (do
          (log/info "Waveform created with ID:" wave-id)
          (let [wave-pid (start-waveform-chain wave-id loop-count)]
            (monitor-limit-switches wave-id wave-pid)
            (log/infof "Waveform chain started for wave-id %s" wave-id)))
        (log/errorf "Failed to create waveform, received invalid wave-id: %s" wave-id))
      (log/error "Failed to create waveform, no wave-id received"))))

;; --- Main Public Function ---

(defn send-commands
  "Processes motor commands, generates a synchronized waveform, and executes it."
  [commands]
  (log/infof "Processing motor commands: %s" commands)
  ;; Ensure commands are sorted by motor-id to maintain consistent pin order
  (let [sorted-commands (sort-by key commands)
        step-counts (map (comp :total-pulses val) sorted-commands)
        step-pins (motor-step-pins)]

    (log/info "Setting motor directions...")
    (set-direction-pins commands)

    (log/info "Generating synchronized waveform for steps:" step-counts "on pins:" step-pins)
    (let [{:keys [waveform loop-count]} (timing/generate-waveform-chain step-counts step-pins)]
      (do (log/info "loop-count:" loop-count)
        (if (and (seq waveform) (pos? loop-count))
          (create-and-run-wave waveform loop-count)
          (log/info "No movement required (zero pulses or empty waveform)."))))))

(comment
  ;; Example of moving three motors with different step counts.
  ;; This is now possible with the refactored driver.
  (let [commands {0 {:total-pulses 2000, :direction 1} ; Motor 0 moves 2000 steps
                  1 {:total-pulses 1000, :direction 1} ; Motor 1 moves 1000 steps
                  2 {:total-pulses 500,  :direction 0}}] ; Motor 2 moves 500 steps
    (send-commands commands))

  (timing/run-demo [2000 1000 500])
  )
