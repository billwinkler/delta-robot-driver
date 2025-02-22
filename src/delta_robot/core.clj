(ns delta-robot.core
  (:require [clojure.edn :as edn]
            [clojure.java.io :as io]))

;; Read and update config (note how base/effector radii are computed from the edges)
(def config
  (-> (slurp "config.edn")
      edn/read-string
      (as-> cfg (assoc cfg :base-radius (/ (:base-edge cfg) (Math/sqrt 3)))
        (assoc cfg :effector-radius (* (/ (Math/sqrt 3) 3) (:effector-edge cfg))))))

;; We maintain current motor angles in an atom (in degrees)
;; Assume initial state is fully retracted
(def current-angles (atom (vec (repeat 3 (:min-angle config)))))


;; Convert an angular difference (in degrees) to motor pulses.
(defn deg->pulses [deg]
  (Math/round (* (/ deg 360.0) (:steps-per-rev config))))

(defn clamp [x min-val max-val]
  (max min-val (min x max-val)))

(defn compute-step-commands
  "For a given target (x, y, z), compute a sequence of motor commands.
   Returns a map with the commands and the new angles."
  [x y z]
  (println "DEBUG: Computing step commands for target:" [x y z])
  (println "DEBUG: Starting current-angles:" @current-angles)
  (let [desired-angles (mapv #(delta-angle % x y z config) [0 1 2])
        clamped-angles (mapv #(clamp % (:min-angle config) (:max-angle config))
                             desired-angles)
        angle-deltas (mapv - clamped-angles @current-angles)
        commands (map-indexed (fn [i delta]
                                (let [pulses    (Math/abs (deg->pulses delta))
                                      direction (if (pos? delta) 0 1)]
                                  (println "DEBUG: Motor" i "move:" delta "deg =>"
                                           pulses "pulses, direction:" direction)
                                  {:motor-number i
                                   :total-pulses pulses
                                   :direction direction}))
                              angle-deltas)]
    (println "DEBUG: Computed new angles:" clamped-angles)
    {:commands commands
     :new-angles clamped-angles}))

;; Main function: given a target (x,y,z), compute pulses for each motor.
(defn compute-steps
  "Compute the number of pulses required for each motor to move the effector
   from its current position to (x,y,z). Returns a vector of three pulse counts."
  [x y z]
  (let [desired-angles (mapv #(delta-angle % x y z config) [0 1 2])
        ;; Enforce the physical limits for each motor's angle:
        clamped-angles (mapv #(clamp % (:min-angle config) (:max-angle config))
                             desired-angles)
        ;; Calculate the difference between the clamped target angles and current angles.
        angle-deltas (mapv - clamped-angles @current-angles)]
    ;; Convert each angular difference to motor pulses.
    (mapv deg->pulses angle-deltas)))

(comment
  (compute-steps 10 10 10)
  (delta-angle 0 0 0 10 config)
  ;; => 1.9850413434129677
  (delta-angle 1 0 0 10 config)
  ;; => 1.9850413434129677
  (delta-angle 1 0 0 100 config)
  (delta-angle 1 0 0 0.01 config)
  (delta-angle 1 0 0 130 config)

  (compute-steps 0 0 0.1)
  ;; => [249 249 249]

  @current-angles

  )
