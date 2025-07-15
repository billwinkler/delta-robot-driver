(ns delta-robot.motion
  (:require [delta-robot.command-driver :refer [send-commands home-motors]]
            [delta-robot.config :as cfg]
            [delta-robot.core :refer [compute-step-commands clamp deg->pulses current-angles]]))

(defn reset []
  (reset! current-angles (vec (repeat 3 (:max-angle cfg/config)))))

(defn home []
  (println "Homing the robot...")
  (home-motors)
  ;; Reset the current angles to the fully retracted value
  (let [{:keys [max-angle]} cfg/config]
    (reset! current-angles (vec (repeat 3 max-angle))))
  (println "Homing complete. Current angles:" @current-angles))

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
    (let [{:keys [commands new-angles]} (compute-step-commands x y z)
          command-map (into {} (map (fn [{:keys [motor-number total-pulses direction]}]
                                      [motor-number {:total-pulses total-pulses :direction direction}])
                                    commands))]
      (println "Sending commands:" commands)
      (send-commands command-map)
      ;; Update state after movement completes.
      (reset! current-angles new-angles)
      ;; Optionally pause before the next move.
      ;; (Thread/sleep 500)
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

