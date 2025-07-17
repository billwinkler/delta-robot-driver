(ns delta-robot.motion
  (:require [delta-robot.command-driver :refer [send-commands home-motors busy-wait]]
            [delta-robot.gripper :refer [open close grip]]
            [delta-robot.config :as cfg]
            [delta-robot.core :refer [compute-step-commands clamp deg->pulses current-angles]]
            [clojure.tools.logging :as log]))

(defn reset []
  (reset! current-angles (vec (repeat 3 (:max-angle cfg/config)))))

(defn nudge []
      ;; 0 is down
  (let [commands {0 {:total-pulses 100, :direction 0} 
                  1 {:total-pulses 100, :direction 0} 
                  2 {:total-pulses 100, :direction 0}}]
    (send-commands commands)))

(defn home []
  (log/info "Homing the robot...")
  (home-motors)
  ;; Reset the current angles to the fully retracted value
  (let [{:keys [max-angle]} cfg/config]
    (reset! current-angles (vec (repeat 3 max-angle))))
  (log/info "Homing complete. Current angles:" @current-angles))

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
      (log/info "Sending commands:" commands)
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
  
  (move-path [[0 0 410]])
  (move-path [[20 20 400]])
  
  (move-path [[70 70 300]])
  (move-path [[60 60 300]])
  (move-path [[50 50 300]])
  (move-path [[40 40 300]])
  (move-path [[20 20 300]])
  (move-path [[10 10 300]])
  (move-path [[-20 -20 300]])
  (move-path [[-30 -30 300]])
  (move-path [[-50 -50 300]])
  (move-path [[-70 -70 300]])
  (move-path [[-80 -80 300]])
  (move-path [[-90 -90 300]])

  (do
    (move-path [[0 -100 300]])
    (busy-wait)
    (move-path [[0 100 300]])
    (busy-wait)
    (move-path [[100 0 300]])
    (busy-wait)
    (move-path [[-100 0 300]]))

  
  (move-path [[0 0 250]])
  (move-path [[45 -30 380]])
  (move-path [[45 -30 420]])
  (grip 30)
  (open)
  (move-path [[0 0 250]])
  (move-path [[-115 25 380]])

  (reset)
  (do
    (move-path [[0 0 250]])
    (Thread/sleep 5000)
    (home)
    (Thread/sleep 3000)
    (nudge)
    (Thread/sleep 2000)
    (nudge)
    (Thread/sleep 2000))
  
  (do
    (do
      (move-path [[0 0 250]])
      (Thread/sleep 5000)
      (home)
      (Thread/sleep 3000)
      (nudge)
      (Thread/sleep 2000)
      (nudge)
      (Thread/sleep 2000))
    (do
      (open)
      (Thread/sleep 3000)
      (move-path [[0 0 250]])
      (Thread/sleep 3000)
      (move-path [[45 -30 380]])
      (Thread/sleep 5000)
      (move-path [[45 -30 420]])
      (Thread/sleep 3000)
      (grip 30)
      )
    (Thread/sleep 3000)
    (do
      (move-path [[0 0 250]])
      (Thread/sleep 3000)
      (move-path [[-115 25 380]])
      (Thread/sleep 3000)
      (move-path [[-115 25 400]])
      (Thread/sleep 2000)
      (open)
      )
    (Thread/sleep 3000)
    (do
      (move-path [[0 0 250]])
      (Thread/sleep 3000)
      (close)
      (do
        (move-path [[0 0 250]])
        (Thread/sleep 5000)
        (home)
        (Thread/sleep 3000)
        (nudge)
        (Thread/sleep 2000)
        (nudge))))


  )

