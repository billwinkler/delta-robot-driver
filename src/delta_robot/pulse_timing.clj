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
  Streaming bounds CB usage at ~2 chunks regardless of move length."
  (:require [delta-robot.config :refer [motor-step-pins]]))

;; --- Constants ---
;; Timings are in integer microseconds, as required by the pigpiod library.

(def high-pulse-us
  "The duration of the 'high' part of a single motor pulse, in microseconds."
  500)

(def min-low-pulse-us
  "The minimum duration of the 'low' part of a single motor pulse, in microseconds.
  This is the duration used by the motor with the most steps."
  500)

(def max-pulses-per-waveform
  "The maximum number of pulses per chunk. With streamed transmission at
  most two chunks are alive at once (playing + queued), so CB usage is
  bounded at roughly 2 * this * ~2 CBs/pulse — far inside pigpiod's
  pool. Larger chunks also widen the timing margin: a chunk plays for
  ~(chunk/2) ms while the next one is created via a few pigs calls."
  256)

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
      (let [;; Total move time is set by the busiest motor.
            total-duration-us (* max-steps (+ high-pulse-us min-low-pulse-us))

            ;; Generate all state-change events (pulse start/end).
            all-events (->> (map (fn [steps pin]
                                   (when-not (zero? steps)
                                     (let [;; Coerce to double to prevent creating a Ratio,
                                           ;; which Math/round cannot handle.
                                           step-duration-us (/ (double total-duration-us) steps)]
                                       (mapcat (fn [i]
                                                 (let [start-time (* i step-duration-us)
                                                       high-end-time (+ start-time high-pulse-us)]
                                                   [[(long (Math/round start-time)) pin :high]
                                                    [(long (Math/round high-end-time)) pin :low]]))
                                               (range steps)))))
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
