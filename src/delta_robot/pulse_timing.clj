(ns delta-robot.timing)

;; --- Part 1: High-Precision Duration Calculation (Nanoseconds) ---

(def high-pulse-ns
  "The duration of the 'high' part of a single motor pulse, in nanoseconds."
  500)

(def min-low-pulse-ns
  "The minimum duration of the 'low' part of a single motor pulse, in nanoseconds."
  500)

(defn calculate-low-durations
  "Calculates the required low-pulse duration (in ns) for each motor for synchronized movement.
  This function provides a high-precision theoretical value."
  [step-counts]
  (let [max-steps (apply max step-counts)
        total-duration-ns (if (zero? max-steps)
                            0
                            (* max-steps (+ high-pulse-ns min-low-pulse-ns)))]
    (map
      (fn [steps]
        (if (zero? steps)
          total-duration-ns
          (- (/ total-duration-ns steps) high-pulse-ns)))
      step-counts)))

;; --- Example Usage for Part 1 ---

(def motor-steps-1 [1000 900 250])
(def durations-1 (calculate-low-durations motor-steps-1))
(println "--- Part 1: Nanosecond Durations ---")
(println "For steps:" motor-steps-1)
(println "Calculated low-pulse durations (ns):" durations-1)
(println (apply str (repeat 50 "-")))


;; --- Part 2: pigpiod Waveform Generation (Microseconds) ---
;; This part generates a concrete, interleaved waveform suitable for use
;; with the pigpiod library's wave chaining functionality.

(def high-pulse-us
  "High pulse duration in integer microseconds for pigpiod."
  500)

(def min-low-pulse-us
  "Minimum low pulse duration in integer microseconds for pigpiod."
  500)

(defn- gcd
  "Calculates the greatest common divisor of two numbers."
  [a b]
  (if (zero? b) a (recur b (mod a b))))

(defn- gcd-coll
  "Calculates the greatest common divisor for a collection of numbers, ignoring zeros."
  [coll]
  (let [non-zero (remove zero? coll)]
    (if (empty? non-zero) 0 (reduce gcd non-zero))))

(defn generate-waveforms
  "Generates a repeatable, interleaved waveform for pigpiod.

  Takes motor steps and a corresponding list of GPIO pin numbers.
  Returns a map containing:
  - :waveform    A list of pigpiod pulses `[gpio-on-mask gpio-off-mask delay-us]`.
  - :loop-count  The number of times the waveform should be looped.

  This allows for efficient, synchronized control using `pigpiod.wave_chain`."
  [step-counts gpio-pins]
  (let [;; --- 1. Initial calculations in microseconds ---
        max-steps (apply max step-counts)
        loop-count (gcd-coll step-counts)]

    (if (or (zero? max-steps) (zero? loop-count))
      {:waveform [] :loop-count 0} ; No movement, return empty wave.
      (let [total-duration-us (* max-steps (+ high-pulse-us min-low-pulse-us))
            ;; Coerce to double to ensure floating point division for accuracy.
            base-duration-us (/ (double total-duration-us) loop-count)

            ;; --- 2. Generate all state-change events for the base waveform ---
            all-events (->> (map (fn [steps pin]
                                   (when-not (zero? steps)
                                     ;; Coerce to double to prevent creating a Ratio type, which Math/round cannot handle.
                                     (let [step-duration-us (/ (double total-duration-us) steps)
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

            ;; --- 3. Group events by time and get a sorted list of unique times ---
            events-by-time (group-by first all-events)
            sorted-times (sort (keys events-by-time))]

        ;; --- 4. Process events into a pigpiod waveform ---
        {:waveform (loop [times sorted-times
                          pulses []]
                     (if-let [current-time (first times)]
                       ;; The delay for the current pulse is the time until the next event.
                       (let [;; Round the final duration for accuracy.
                             next-time (or (second times) (long (Math/round base-duration-us)))
                             delay-us (- next-time current-time)
                             events-now (get events-by-time current-time)

                             ;; Create bitmasks for pins to turn on or off at this instant.
                             gpio-on (reduce bit-or 0 (for [[_ p s] events-now :when (= s :high)] (bit-shift-left 1 p)))
                             gpio-off (reduce bit-or 0 (for [[_ p s] events-now :when (= s :low)] (bit-shift-left 1 p)))]

                         ;; A pulse is (on, off, delay). The state change happens, then the delay.
                         ;; We only add a pulse if there's a delay to wait.
                         (if (> delay-us 0)
                           (recur (next times) (conj pulses [gpio-on gpio-off delay-us]))
                           (recur (next times) pulses)))
                       pulses))
         :loop-count loop-count}))))

;; --- Example Usage for Part 2 ---

(def motor-pins [4 17 27]) ; Example GPIO pins for motors 1, 2, 3

(println "\n--- Part 2: pigpiod Waveform Generation ---")
(println "For steps:" motor-steps-1 "and pins:" motor-pins)

(let [{:keys [waveform loop-count]} (generate-waveforms motor-steps-1 motor-pins)]
  (println "Generated a base waveform with" (count waveform) "pulses.")
  (println "This waveform should be looped" loop-count "times.")
  (println "First 5 pulses [on-mask off-mask delay-us]:")
  (->> waveform
       (take 10)
       (run! println))
  (println "..."))

;; Expected output for [1000 500 250] on pins [4 17 27]:
;; on-mask for pin 4 is 16 (1<<4)
;; on-mask for pin 17 is 131072 (1<<17)
;; on-mask for pin 27 is 134217728 (1<<27)
;;
;; t=0: all motors on. mask = 16+131072+134217728 = 134348816. delay=1us.
;; t=1: all motors off. mask = 134348816. delay=1us.
;; t=2: M1 on. mask=16. delay=1us.
;; t=3: M1 off. mask=16. delay=1us.
;; t=4: M1,M2 on. mask=16+131072=131088. delay=1us.
