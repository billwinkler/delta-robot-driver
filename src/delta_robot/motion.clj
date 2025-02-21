(ns delta-robot.motion
  (:require [delta-robot.command-driver :refer [send-commands]]
            [delta-robot.core :refer [delta-angle clamp deg->pulses config current-angles]]))

(defn home []
  (let [max-angle 85
        min-angle -28
        ;; Calculate the angular difference (should be negative)
        delta (- min-angle max-angle)  ; -113° in this case
        ;; Compute the absolute number of pulses required:
        pulses (Math/abs (deg->pulses delta))
        ;; For each motor, set the retract direction (assumed to be 1)
        home-commands (for [i (range 3)]
                        {:motor-number i
                         :total-pulses pulses
                         :direction 1})]
    (println "Homing: sending home command:" home-commands)
    (send-commands home-commands)
    ;; Allow some time for the motors to move and for the limit switches to halt them
    (Thread/sleep 2000)
    ;; Reset the current angles to the fully retracted value
    (reset! current-angles (vec (repeat 3 min-angle)))
    (println "Homing complete. Current angles:" @current-angles)))


(defn compute-step-commands
  "For a given target (x, y, z), compute a sequence of motor commands.
   Returns a map with the commands and the new angles."
  [x y z]
  (let [desired-angles (mapv #(delta-angle % x y z config) [0 1 2])
        clamped-angles (mapv #(clamp % (:min-angle config) (:max-angle config))
                             desired-angles)
        angle-deltas (mapv - clamped-angles @current-angles)
        commands (map-indexed (fn [i delta]
                                (let [pulses    (Math/abs (deg->pulses delta))
                                      direction (if (pos? delta) 0 1)]
                                  {:motor-number i
                                   :total-pulses pulses
                                   :direction direction}))
                              angle-deltas)]
    {:commands commands
     :new-angles clamped-angles}))

(def moves
  "A sequence of target coordinates (x y z) for the effector."
  [[50 50 50]
   [0 0 50]])

(defn move-path [moves]
  "Iterate over a sequence of target positions, sending the corresponding motor commands and updating the state."
  (doseq [[x y z] moves]
    (let [{:keys [commands new-angles]} (compute-step-commands x y z)]
      (println "Sending commands:" commands)
      (send-commands commands)
      ;; Update state after movement completes.
      (reset! current-angles new-angles)
      ;; Optionally pause before the next move.
      (Thread/sleep 1000))))

(move-path moves)
;; => java.lang.Exception: Target position unreachable delta-robot.motion /Users/billwinkler/dev/delta-arm-v2/babashka/src/delta_robot/core.clj:37:7
