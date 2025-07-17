(ns delta-robot.gripper
  (:require [delta-robot.command-driver :refer [execute-pigs-cmd]]
            [delta-robot.config :refer [effector-servo-pin]]))
(def close-pulse 600)
(def open-pulse 1700)

(defn- servo
  [pulse-width]
  (execute-pigs-cmd "servo" (effector-servo-pin) pulse-width))

(defn open []
  (servo open-pulse)
  (Thread/sleep 2000)
  (servo 0))

(defn close []
  (servo close-pulse)
  (Thread/sleep 2000)
  (servo 0))

(defn grip [mm]
  (let [open-width 40
        target (/ mm open-width)
        pulses (int (+ close-pulse
                       (*
                        target
                        (- open-pulse close-pulse))))]
    (println "grip pulses" (int pulses))
    (servo pulses)
    (Thread/sleep 2000)
    (servo 0))
  )

(comment
  (open)
  (close)
  (grip 30)


  )

