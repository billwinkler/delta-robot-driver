(ns delta-robot.command-driver
  "Drives the steppers through pigpiod via the pigs CLI, streaming each
  move as a sequence of small waveforms (wvtxm one-shot-sync ping-pong).

  send-commands is SYNCHRONOUS: it returns only after the move finishes
  (or aborts on a limit switch), and every wave it created has been
  deleted. This fixes two bugs in the previous design:
  1. CB exhaustion — the old wvcha chain needed every chunk alive at
     once; gcd=1 moves exhausted pigpio's DMA control-block pool at
     wvcre. Streaming keeps at most 2 waves alive (playing + queued).
  2. Deletion race — a fire-and-forget future deleted wave ids after
     transmission, racing the next move's wvcre (which re-uses ids
     0,1,2...). No background deletion exists any more."
  (:require [babashka.process :refer [sh]]
            [clojure.string :as str]
            [clojure.tools.logging :as log]
            [delta-robot.config :refer [motor-step-pins motor-direction-pins limit-switch-pins]]
            [delta-robot.pulse-timing :as timing]))

;; --- Utility Functions ---

(defn execute-pigs-cmd
  "Executes a pigpio command via the 'pigs' utility and returns trimmed output."
  [& args]
  (try
    (let [{:keys [out err exit]} (apply sh "pigs" args)]
      (if (zero? exit)
        (str/trim out)
        (do
          (log/errorf "Failed to execute pigs command %s: exit code %d, error: %s" args exit err)
          (throw (ex-info (str "pigpio command failed: " err) {:command args :exit exit :error err})))))
    (catch Exception e
      (log/errorf "Exception executing pigs command %s: %s" args (.getMessage e))
      (throw e))))

(defn busy-wait
  "Blocks until no waveform is being transmitted. send-commands is now
  synchronous, so this returns immediately after it — kept for callers
  (motion.clj) and as a safety net."
  []
  (while (= "1" (execute-pigs-cmd "wvbsy"))
    (Thread/sleep 5)))

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

(defn- limit-triggered?
  "True when any motor that is moving up (direction 1) has hit its limit
  switch (active low)."
  [commands]
  (try
    (let [pin-values (mapv #(execute-pigs-cmd "r" (str %)) (limit-switch-pins))]
      (boolean (some (fn [[motor-id {:keys [direction total-pulses]}]]
                       (and (pos? (or total-pulses 0))
                            (= 1 direction)
                            (= "0" (nth pin-values motor-id nil))))
                     commands)))
    (catch Exception e
      (log/errorf "Error reading limit switches: %s" (.getMessage e))
      false)))

(defn- is-motor-at-limit?
  "Checks if a motor is being commanded up while its limit switch is pressed."
  [motor-id direction]
  (if (= direction 1)
    (let [limit-pin (nth (limit-switch-pins) motor-id)
          limit-value (execute-pigs-cmd "r" (str limit-pin))]
      (= "0" limit-value))
    false))

;; --- Streamed Waveform Transmission ---

(defn- create-wave!
  "Adds a pulse chunk to pigpiod and creates a wave from it. Returns the
  wave id."
  [pulse-chunk]
  (apply execute-pigs-cmd "wvag" (map str (flatten pulse-chunk)))
  (Integer/parseInt (execute-pigs-cmd "wvcre")))

(defn- abort!
  "Halts transmission and deletes all waves."
  [reason]
  (log/warnf "Aborting move: %s" reason)
  (execute-pigs-cmd "wvhlt")
  (clear-waveforms)
  :aborted)

(defn- wait-until-playing
  "Polls until the queued wave `wid` starts playing (i.e. its predecessor
  finished), checking limit switches. Returns :ok, :done (transmission
  already past wid), or :limit."
  [wid commands]
  (loop []
    (if (limit-triggered? commands)
      :limit
      (let [at (execute-pigs-cmd "wvtat")]
        (cond
          (= at (str wid)) :ok
          ;; 9999 = no wave transmitting: we were slow and the whole
          ;; queue drained. 9998 = transmitted wave was deleted.
          (contains? #{"9999" "9998"} at) :done
          :else (do (Thread/sleep 2) (recur)))))))

(defn- final-wait
  "Blocks until transmission finishes, checking limit switches.
  Returns :ok or :limit."
  [commands]
  (loop []
    (cond
      (limit-triggered? commands) :limit
      (not= "1" (execute-pigs-cmd "wvbsy")) :ok
      :else (do (Thread/sleep 2) (recur)))))

(defn- stream-chunks!
  "Streams pulse chunks through pigpiod: at most two waves are alive at a
  time (playing + queued via one-shot-sync). Deletes each wave once its
  successor is playing. Returns :ok or :aborted."
  [chunks commands]
  (clear-waveforms)
  (let [first-wid (create-wave! (first chunks))]
    (execute-pigs-cmd "wvtxm" (str first-wid) "0") ; one-shot: start now
    (loop [prev-wid first-wid
           remaining (rest chunks)]
      (if (seq remaining)
        (let [wid (create-wave! (first remaining))]
          (execute-pigs-cmd "wvtxm" (str wid) "2") ; one-shot-sync: queue
          (case (wait-until-playing wid commands)
            :limit (abort! "limit switch triggered")
            (do (execute-pigs-cmd "wvdel" (str prev-wid))
                (recur wid (rest remaining)))))
        ;; last wave queued/playing — drain and clean up
        (case (final-wait commands)
          :limit (abort! "limit switch triggered")
          (do (execute-pigs-cmd "wvdel" (str prev-wid))
              :ok))))))

(def max-chain-pulses
  "Moves up to this many total pulses are pre-built and played as ONE
  wvcha chain — a single motor start per move. Streaming (above) starts
  the motors cold at full step rate at EVERY chunk boundary; with no
  acceleration ramp those restarts shed steps against the arm's inertia
  (measured: ~70mm of position drift over 50 moves in the 2026-07-03
  hangar audit vs the 2025 baseline of 15-40mm per afternoon).
  CB budget: 2400 pulses * ~3 CBs ~= 7k of pigpiod's ~25k pool."
  2400)

(defn- chain-chunks!
  "Pre-creates every chunk's wave and plays them as one seamless wvcha
  chain. Suitable only for moves within max-chain-pulses. Returns :ok
  or :aborted."
  [chunks commands]
  (clear-waveforms)
  (let [wids (mapv create-wave! chunks)]
    (apply execute-pigs-cmd "wvcha" (map str wids))
    (case (final-wait commands)
      :limit (abort! "limit switch triggered")
      (do (doseq [wid wids]
            (execute-pigs-cmd "wvdel" (str wid)))
          :ok))))

;; --- Main Public Function ---

(defn send-commands
  "Processes motor commands, generates a synchronized pulse stream, and
  transmits it. Blocks until the move completes. Returns :ok, or
  :aborted when a limit switch stopped the move."
  [commands]
  (log/infof "Processing motor commands: %s" commands)
  ;; Prevent motors from moving up if they are already at the limit switch.
  (let [checked-commands (into {}
                               (map (fn [[motor-number {:keys [direction] :as command}]]
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

    (log/info "Generating pulse chunks for steps:" step-counts "on pins:" step-pins)
    (let [chunks (timing/generate-pulse-chunks step-counts step-pins)
          total-pulses (reduce + 0 (map count chunks))]
      (if (seq chunks)
        (let [chained? (<= total-pulses max-chain-pulses)
              result (if chained?
                       (chain-chunks! chunks checked-commands)
                       (stream-chunks! chunks checked-commands))]
          (log/infof "Move finished: %s (%d chunks, %d pulses, %s)"
                     result (count chunks) total-pulses
                     (if chained? "chained" "streamed"))
          result)
        (do (log/info "No movement required (zero pulses or empty waveform).")
            :ok)))))

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
  (let [commands {0 {:total-pulses 500, :direction 1}
                  1 {:total-pulses 501, :direction 1}
                  2 {:total-pulses 502, :direction 1}}]
    (send-commands commands))

  ;; 0 is down
  (let [commands {0 {:total-pulses 500, :direction 0}
                  1 {:total-pulses 500, :direction 0}
                  2 {:total-pulses 500, :direction 0}}]
    (send-commands commands))

  (home-motors)

  (timing/run-demo [100 101 102])
  (timing/run-demo [2000 1900 1800])
  )
