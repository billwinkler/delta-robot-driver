(ns delta-robot.motion
  (:require [delta-robot.command-driver :refer [send-commands]]
            [delta-robot.config :as cfg]
            [delta-robot.core :refer [compute-step-commands clamp deg->pulses current-angles]]))

(defn reset []
  (reset! current-angles (vec (repeat 3 (:max-angle cfg/config)))))

(defn home []
  (let [{:keys [min-angle max-angle]} cfg/config
        ;; Calculate the angular difference (should be negative)
        delta (- min-angle max-angle)   
        _ (println "delta" delta)
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
    (reset! current-angles (vec (repeat 3 max-angle)))
    (println "Homing complete. Current angles:" @current-angles)))

(def moves
  "A sequence of target coordinates (x y z) for the effector."
  [[0 0 275]
   [50 50 275]
   [-50 50 275]
   [-50 -50 275]
   [50 -50 275]
   [50 50 275]
   [0 0 275]
   [0 0 217]])

(defn move-path [moves]
  "Iterate over a sequence of target positions, sending the corresponding motor commands and updating the state."
  (doseq [[x y z] moves]
    (let [{:keys [commands new-angles]} (compute-step-commands x y z)]
      (println "Sending commands:" commands)
      (send-commands commands)
      ;; Update state after movement completes.
      (reset! current-angles new-angles)
      ;; Optionally pause before the next move.
;;      (Thread/sleep 500)
      )))

(comment
  (reset)
  (let [[x y z] [100 100 300]
        {:keys [commands new-angles]} (compute-step-commands x y z)]
      (println "Sending commands:" commands)
      (send-commands commands)
      ;; Update state after movement completes.
      (reset! current-angles new-angles)
      )
  (compute-step-commands 0 0 400)
  (dotimes [n 3]
    (move-path moves))
  
  (move-path [[0 0 400]])
  (home)
)

