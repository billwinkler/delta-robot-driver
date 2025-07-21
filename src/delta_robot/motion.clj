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
    (send-commands commands)
    (busy-wait)))

(defn home []
  (log/info "Homing the robot...")
  (home-motors)
  (busy-wait)
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

(defn move-to [x y z]
  "Move the effector to a single target position (x y z)."
  (let [{:keys [commands new-angles]} (compute-step-commands x y z)
        command-map (into {} (map (fn [{:keys [motor-number total-pulses direction]}]
                                    [motor-number {:total-pulses total-pulses :direction direction}])
                                  commands))]
    (log/info "Sending commands:" commands)
    (send-commands command-map)
    (busy-wait)
    ;; Update state after movement completes.
    (reset! current-angles new-angles)))

(defn move-path [moves]
  "Iterate over a sequence of target positions, sending the corresponding motor commands and updating the state."
  (doseq [[x y z] moves]
    (move-to x y z)
    ;; Optionally pause before the next move.
    ;; (Thread/sleep 500)
    ))

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
  
  (move-to 0 0 410)
  (move-to 20 20 400)
  
  (move-to 70 70 300)
  (move-to 60 60 300)
  (move-to 50 50 300)
  (move-to 40 40 300)
  (move-to 20 20 300)
  (move-to 10 10 300)
  (move-to -20 -20 300)
  (move-to -30 -30 300)
  (move-to -50 -50 300)
  (move-to -70 -70 300)
  (move-to -80 -80 300)
  (move-to -90 -90 300)

  (do
    (move-to 0 -100 300)
    (move-to 0 100 300)
    (move-to 100 0 300)
    (move-to -100 0 300))
  
  (move-to 0 0 250)
  (move-to 45 -30 380)
  (move-to 45 -30 420)
  (grip 30)
  (open)
  (move-to 0 0 250)
  (move-to -115 25 380)

  (reset)
  (do
    (move-to 0 0 250)
    (home)
    (nudge)
    (nudge))
  
  (do
    (home)
    (do
      (move-to 0 0 250)
      (move-to 45 -30 380)
      (open)
      (Thread/sleep 1000)
      (move-to 45 -30 425)
      (grip 30))
    (Thread/sleep 1000)
    (do
      (move-to 0 0 250)
      (move-to -100 25 250)
      (move-to -100 25 350)
      (Thread/sleep 1000)
      (open)
      (Thread/sleep 1000)
      (move-to -100 25 350)
      (move-to -100 25 250)
      (move-to 0 0 250))
    (Thread/sleep 1000)
    (home)
    (close))


  )

