(ns delta-robot.pulse-timing
  "Calculates and generates a repeatable, interleaved pigpiod waveform
  for synchronized control of multiple stepper motors."
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
  "The maximum number of pulses to include in a single pigpiod waveform.
  This is a soft limit to avoid exceeding pigpiod's internal memory limits
  for a single waveform. Larger pulse trains will be split into a chain
  of multiple waveforms."
  100)

;; --- Helper Functions ---

(defn- gcd
  "Calculates the greatest common divisor of two numbers using the Euclidean algorithm."
  [a b]
  (if (zero? b) a (recur b (mod a b))))

(defn- gcd-coll
  "Calculates the greatest common divisor for a collection of numbers.
  It correctly handles and ignores any zero values in the collection."
  [coll]
  (let [non-zero (remove zero? coll)]
    (if (empty? non-zero) 0 (reduce gcd non-zero))))

;; --- Core Waveform Generation ---

(defn generate-waveform-chain
  "Generates a repeatable, interleaved waveform for pigpiod's wave chaining.
  If the generated waveform is too large, it is split into a sequence of
  smaller waveforms that can be chained together."
  [step-counts gpio-pins]
    (let [;; 1. Find the GCD of the steps to determine the smallest repeating pattern.
        ;;    This count is how many times the base waveform will be chained.
        loop-count (gcd-coll step-counts)
        max-steps (apply max step-counts)]

    ;; If there's no movement, return an empty waveform that does nothing.
    (if (or (zero? max-steps) (zero? loop-count))
      {:waveforms [] :loop-count 0}

      (let [;; 2. Calculate total move time based on the busiest motor.
            total-duration-us (* max-steps (+ high-pulse-us min-low-pulse-us))
            ;; The duration of our small, repeatable base waveform.
            base-duration-us (/ (double total-duration-us) loop-count)

            ;; 3. Generate all state-change events (pulse start/end) for the base waveform.
            all-events (->> (map (fn [steps pin]
                                   (when-not (zero? steps)
                                     (let [;; Coerce to double to prevent creating a Ratio, which Math/round cannot handle.
                                           step-duration-us (/ (double total-duration-us) steps)
                                           base-steps (/ steps loop-count)]
                                       (mapcat (fn [i]
                                                 (let [start-time (* i step-duration-us)
                                                       high-end-time (+ start-time high-pulse-us)]
                                                   [[(long (Math/round start-time)) pin :high]
                                                    [(long (Math/round high-end-time)) pin :low]]))
                                               (range base-steps)))))
                                 step-counts gpio-pins)
                            (apply concat)
                            (sort-by first))

            ;; 4. Group events by their exact time of occurrence.
            events-by-time (group-by first all-events)
            sorted-times (sort (keys events-by-time))

            ;; 5. Process the timed events into a concrete pigpiod waveform list.
            all-pulses (loop [times sorted-times
                              pulses []]
                           (if-let [current-time (first times)]
                             (let [;; The delay for the current pulse is the time until the next event.
                                   next-time (or (second times) (long (Math/round base-duration-us)))
                                   delay-us (- next-time current-time)
                                   events-now (get events-by-time current-time)

                                   ;; Create bitmasks for all pins that turn on or off at this instant.
                                   gpio-on (reduce bit-or 0 (for [[_ p s] events-now :when (= s :high)] (bit-shift-left 1 p)))
                                   gpio-off (reduce bit-or 0 (for [[_ p s] events-now :when (= s :low)] (bit-shift-left 1 p)))]

                               ;; A pulse is (on, off, delay). The state change happens, then the delay.
                               ;; We only add a pulse if there's a non-zero delay to wait.
                               (if (> delay-us 0)
                                 (recur (next times) (conj pulses [gpio-on gpio-off delay-us]))
                                 (recur (next times) pulses)))
                             pulses))]
        ;; 6. Split the waveform into manageable chunks for pigpiod.
        {:waveforms (mapv vec (partition-all max-pulses-per-waveform all-pulses))
         :loop-count loop-count}))))
;; --- Example Usage ---

;; (def motor-steps [1000 500 250])

(defn run-demo [steps]
  (println "--- Final pigpiod Waveform Generation ---")
  (println "For steps:" steps "and pins:" (motor-step-pins))

  (let [{:keys [waveforms loop-count]} (generate-waveform-chain steps (motor-step-pins))]
    (println "Generated" (count waveforms) "base waveforms to be chained.")
    (println "Total pulses in base:" (reduce + 0 (map count waveforms)))
    (println "This waveform chain should be looped" loop-count "times.")
    (when (seq (first waveforms))
      (println "\nFirst 5 pulses of the first waveform [on-mask off-mask delay-us]:")
      (->> (first waveforms)
           (take 5)
           (run! println))
      (println "..."))))

;; (run-demo [1000 500 250])


;; Expected output for [1000 500 250] on pins [4 17 27]:
;; on-mask for pin 4 is 16 (1<<4)
;; on-mask for pin 17 is 131072 (1<<17)
;; on-mask for pin 27 is 134217728 (1<<27)
;;
;; t=0: all motors on.   mask = 16+131072+134217728 = 134348816. delay=1us.
;; t=1: all motors off. mask = 134348816. delay=1us.
;; t=2: M1 on.          mask = 16. delay=1us.
;; t=3: M1 off.         mask = 16. delay=1us.
;; t=4: M1,M2 on.       mask = 16+131072=131088. delay=1us.
