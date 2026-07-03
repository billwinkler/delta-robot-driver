(ns delta-robot.pulse-timing-test
  "Pure tests for full-move pulse-chunk generation (streamed transmission).
  The old gcd/wvcha design exhausted pigpio's DMA control-block pool on
  gcd=1 moves; the new design generates the FULL move as a lazy sequence
  of bounded chunks, transmitted one or two at a time."
  (:require [clojure.test :refer [deftest is testing]]
            [delta-robot.pulse-timing :as timing]))

(defn- edge-count
  "Number of pulses across all chunks whose on- (or off-) mask includes pin."
  [chunks pin edge]
  (reduce +
          (for [chunk chunks
                [on-mask off-mask _delay] chunk]
            (if (bit-test (case edge :on on-mask :off off-mask) pin) 1 0))))

(defn- total-delay-us [chunks]
  (reduce + (for [chunk chunks [_ _ delay] chunk] delay)))

(def pins [17 18 19])

(deftest gcd-1-move-generates-full-move
  (testing "the pathological gcd=1 case that broke the old design"
    (let [steps [500 501 502]
          chunks (timing/generate-pulse-chunks steps pins)]
      (testing "every motor gets exactly its step count in high edges"
        (doseq [[s p] (map vector steps pins)]
          (is (= s (edge-count chunks p :on)) (str "on-edges pin " p))
          (is (= s (edge-count chunks p :off)) (str "off-edges pin " p))))
      (testing "total duration is max-steps * (high + low)"
        (is (= (* 502 (+ timing/high-pulse-us timing/min-low-pulse-us))
               (total-delay-us chunks))))
      (testing "every chunk is bounded (CB budget)"
        (doseq [chunk chunks]
          (is (<= (count chunk) timing/max-pulses-per-waveform)))))))

(deftest large-gcd-move-still-complete
  (testing "steps sharing a large gcd — previously the wvcha loop case"
    (let [steps [1000 500 250]
          chunks (timing/generate-pulse-chunks steps pins)]
      (doseq [[s p] (map vector steps pins)]
        (is (= s (edge-count chunks p :on)) (str "on-edges pin " p)))
      (is (= (* 1000 (+ timing/high-pulse-us timing/min-low-pulse-us))
             (total-delay-us chunks))))))

(deftest zero-movement-yields-no-chunks
  (is (empty? (timing/generate-pulse-chunks [0 0 0] pins))))

(deftest zero-step-motor-gets-no-edges
  (let [chunks (timing/generate-pulse-chunks [10 0 5] pins)]
    (is (= 10 (edge-count chunks 17 :on)))
    (is (zero? (edge-count chunks 18 :on)))
    (is (zero? (edge-count chunks 18 :off)))
    (is (= 5 (edge-count chunks 19 :on)))))

(deftest chunks-fit-pigs-arg-limit
  (testing "pigs parses at most 512 command-line params; each pulse is 3
    numbers (bench-verified on the Pi: 256-pulse chunks fail wvag)"
    (is (<= (* 3 timing/max-pulses-per-waveform) 510))))

(deftest delays-are-positive
  (testing "pigpiod rejects zero-delay pulses; every emitted pulse must wait"
    (let [chunks (timing/generate-pulse-chunks [500 501 502] pins)]
      (doseq [chunk chunks [_ _ delay] chunk]
        (is (pos? delay))))))
