(ns delta-robot.pulse-timing
  "Calculates interleaved pigpiod pulses for synchronized control of
  multiple stepper motors, delivered as a sequence of bounded chunks
  for STREAMED transmission (wvtxm one-shot-sync ping-pong).

  History: the previous design found the gcd of the step counts, built
  one 'base' waveform, and looped it via a wvcha chain. That collapsed
  for real IK output — arbitrary step counts like [500 501 502] have
  gcd 1, so the 'base' waveform was the entire move and every chunk had
  to exist simultaneously for the chain, exhausting pigpio's DMA
  control-block pool (~25k CBs) at wvcre ('No more CBs for waveform').
  Streaming bounds CB usage at ~2 chunks regardless of move length.

  2026-07-03 hangar audits: cold-starting the motors at full step rate
  sheds steps (~55-70mm dead-reckoning drift over 50 moves). Steps are
  now timed on a TRAPEZOIDAL velocity profile — accelerate from
  :min-frequency, cruise, decelerate — using the config parameters
  that have existed since 2021 but were never honored."
  (:require [delta-robot.config :as cfg :refer [motor-step-pins]]))

;; --- Constants ---
;; Timings are in integer microseconds, as required by the pigpiod library.

(defn- profile
  "Velocity-profile parameters from config. Cruise is clamped to 1 kHz
  (the historical fixed rate) until the arm's dynamics at higher speeds
  are verified — raise the clamp deliberately, not by accident."
  []
  {:min-freq (double (or (:min-frequency cfg/config) 500.0))
   :max-freq (min 1000.0 (double (or (:max-frequency cfg/config) 1000.0)))
   :accel    (long (or (:acceleration-pulses cfg/config) 150))})

(defn- step-periods
  "Per-step periods (us) for S steps under a trapezoidal profile:
  linear frequency ramp over the accel span, cruise, symmetric ramp
  down. The accel span shrinks to S/3 for short moves."
  [S {:keys [min-freq max-freq accel]}]
  (let [A (max 1 (min accel (quot S 3)))
        freq (fn [k]
               (cond
                 (< k A)        (+ min-freq (* (- max-freq min-freq) (/ (double (inc k)) A)))
                 (>= k (- S A)) (+ min-freq (* (- max-freq min-freq) (/ (double (- S k)) A)))
                 :else          max-freq))]
    (mapv #(/ 1e6 (freq %)) (range S))))

(def max-pulses-per-waveform
  "The maximum number of pulses per chunk. Two constraints:
  - pigs parses at most 512 command-line parameters, and each pulse is
    3 numbers, so a chunk must stay under ~170 pulses (bench-verified:
    256 pulses => 768 args => 'wvag: bad parameter').
  - With streamed transmission at most two chunks are alive at once
    (playing + queued), so CB usage is bounded at ~2 * this * ~2
    CBs/pulse — far inside pigpiod's pool. A chunk plays for ~(chunk/2)
    ms, the margin during which the next chunk is created and queued."
  150)

;; --- Core Pulse Generation ---

(defn generate-pulse-chunks
  "Generates the FULL move as interleaved pigpiod pulses, partitioned
  into chunks of at most max-pulses-per-waveform. Each pulse is
  [gpio-on-mask gpio-off-mask delay-us]. Returns [] when no movement.

  Each motor's steps are spread evenly across the whole move duration
  (set by the busiest motor at high+low us per step), so all motors
  start and finish together."
  [step-counts gpio-pins]
  (let [max-steps (apply max step-counts)]
    (if (zero? max-steps)
      []
      (let [periods (step-periods max-steps (profile))
            ;; Busiest-motor step start times: cumulative profile time.
            starts (vec (reductions + 0.0 (pop periods)))
            total-duration-us (long (Math/ceil (+ (peek starts) (peek periods))))

            ;; Each motor's step j maps onto the busiest motor's profile
            ;; at the matching fraction of the move, so all motors
            ;; accelerate, cruise, and decelerate together.
            all-events (->> (map (fn [steps pin]
                                   (when-not (zero? steps)
                                     (mapcat (fn [j]
                                               (let [idx (min (dec max-steps)
                                                              (long (Math/floor (* (+ j 0.5) (/ (double max-steps) steps)))))
                                                     start (nth starts idx)
                                                     high-end (+ start (/ (nth periods idx) 2.0))]
                                                 [[(long (Math/round start)) pin :high]
                                                  [(long (Math/round high-end)) pin :low]]))
                                             (range steps))))
                                 step-counts gpio-pins)
                            (apply concat)
                            (sort-by first))

            ;; Group events by their exact time of occurrence.
            events-by-time (group-by first all-events)
            sorted-times (sort (keys events-by-time))

            ;; Process the timed events into concrete pigpiod pulses.
            all-pulses (loop [times sorted-times
                              pulses []]
                         (if-let [current-time (first times)]
                           (let [;; The delay for the current pulse is the time until
                                 ;; the next event (or the end of the move).
                                 next-time (or (second times) total-duration-us)
                                 delay-us (- next-time current-time)
                                 events-now (get events-by-time current-time)

                                 ;; Bitmasks for all pins changing state at this instant.
                                 gpio-on (reduce bit-or 0 (for [[_ p s] events-now :when (= s :high)] (bit-shift-left 1 p)))
                                 gpio-off (reduce bit-or 0 (for [[_ p s] events-now :when (= s :low)] (bit-shift-left 1 p)))]

                             ;; A pulse is (on, off, delay). Only emit when there is
                             ;; time to wait (distinct integer event times guarantee
                             ;; this except at the very end of the move).
                             (if (pos? delay-us)
                               (recur (next times) (conj pulses [gpio-on gpio-off delay-us]))
                               (recur (next times) pulses)))
                           pulses))]
        (mapv vec (partition-all max-pulses-per-waveform all-pulses))))))

;; --- Example Usage ---

(defn run-demo [steps]
  (println "--- Streamed pigpiod Pulse Generation ---")
  (println "For steps:" steps "and pins:" (motor-step-pins))

  (let [chunks (generate-pulse-chunks steps (motor-step-pins))]
    (println "Generated" (count chunks) "chunks to be streamed.")
    (println "Total pulses:" (reduce + 0 (map count chunks)))
    (when (seq (first chunks))
      (println "\nFirst 5 pulses of the first chunk [on-mask off-mask delay-us]:")
      (->> (first chunks)
           (take 5)
           (run! println))
      (println "..."))))

;; (run-demo [1000 500 250])
;; (run-demo [500 501 502])   ;; gcd=1 — the case that broke the old design
