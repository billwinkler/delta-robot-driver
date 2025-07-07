(ns delta-robot.command-driver
  (:require [babashka.process :refer [sh]]
            [clojure.java.io :as io]
            [clojure.string :as str]
            [delta-robot.config :refer [config]]))

(def min-phase-pulses 10)
(def time-scale-precision 10000)

(defn- estimate-motion-duration [total-pulses accel-pulses decel-pulses]
  (let [min-period (long (/ 1e9 (:max-frequency config)))
        max-period (long (/ 1e9 (:min-frequency config)))
        pulse-overhead (:pulse-overhead-ns config)
        constant-pulses (if (> total-pulses (+ accel-pulses decel-pulses))
                          (- total-pulses accel-pulses decel-pulses)
                          0)]
    (if (<= total-pulses 1)
      0
      (long
       (+ (* (+ max-period min-period) accel-pulses 1/2)
          (* accel-pulses pulse-overhead)
          (* constant-pulses (+ min-period pulse-overhead))
          (* (+ min-period max-period) decel-pulses 1/2)
          (* decel-pulses pulse-overhead))))))

(defn- generate-waveform-commands [motor-id {:keys [total-pulses direction]}]
  (let [motor-keyword (keyword (str "motor" motor-id))
        gpio-step (get-in config [:gpio-pins motor-keyword :step])
        gpio-dir (get-in config [:gpio-pins motor-keyword :dir])
        accel-pulses-cfg (:acceleration-pulses config)
        decel-pulses-cfg (:deceleration-pulses config)
        [accel-pulses decel-pulses]
        (if (> total-pulses (+ accel-pulses-cfg decel-pulses-cfg))
          [accel-pulses-cfg decel-pulses-cfg]
          (let [half-pulses (quot total-pulses 2)]
            [half-pulses (- total-pulses half-pulses)]))
        
        duration (estimate-motion-duration total-pulses accel-pulses decel-pulses)
        
        pulses
        (loop [k 0
               acc []]
          (if (>= k total-pulses)
            acc
            (let [min-period (/ 1e9 (:max-frequency config))
                  max-period (/ 1e9 (:min-frequency config))
                  period-accel (if (and (< k accel-pulses) (> accel-pulses 1))
                                 (- max-period (* (- max-period min-period) (/ k (dec accel-pulses))))
                                 min-period)
                  period-decel (if (and (>= k (- total-pulses decel-pulses)) (> decel-pulses 1))
                                 (let [m (- total-pulses 1 k)]
                                   (+ min-period (* (- max-period min-period) (/ (- decel-pulses 1 m) (dec decel-pulses)))))
                                 min-period)
                  period-k (max period-accel period-decel)
                  delay (long (/ period-k 1000))]
              (recur (inc k) (conj acc {:gpio gpio-step :delay delay})))))]
    {:duration duration
     :pulses pulses
     :direction-commands [(str "pigs w " gpio-dir " " direction)]}))


(defn send-commands [commands]
  (let [waveforms (map-indexed (fn [idx cmd] (generate-waveform-commands idx cmd)) commands)
        max-duration (apply max (map :duration waveforms))
        script (str/join "\n"
                         (concat
                          ["pigs wvas 255 0 # clear any existing waveform"]
                          (mapcat :direction-commands waveforms)
                          (map-indexed (fn [idx wf]
                                         (let [motor-keyword (keyword (str "motor" idx))
                                               gpio (get-in config [:gpio-pins motor-keyword :step])
                                               time-scale (if (pos? (:duration wf))
                                                            (/ (* max-duration (double time-scale-precision)) (:duration wf))
                                                            time-scale-precision)
                                               pulses (:pulses wf)]
                                           (str "pigs wvsc "
                                                (str/join " "
                                                          (map #(long (/ (* (:delay %) time-scale) time-scale-precision)) pulses)))
                                           "\n"
                                           (str "pigs wvcre")
                                           "\n"
                                           (str "pigs wvtx " (inc idx))))
                                       waveforms)
                          [(str "pigs wvag " (str/join " " (map #(inc (key %)) (map-indexed vector commands))))]))
        local-script-path "/tmp/delta_wave.sh"]
    (spit local-script-path script)
    (let [result (sh "bash" local-script-path)]
      (if (zero? (:exit result))
        (println "Command sent successfully!")
        (println "Failed to send command:" (:err result))))))