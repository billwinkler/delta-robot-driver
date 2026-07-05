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

(defn grip-hold
  "Close to mm and KEEP THE PWM ON — the servo holds clamping force
  until a later open/close/grip call changes it.

  The plain grip's (servo 0) de-energizes the servo 2 s after
  closing: the jaws stay put by gear friction but exert ZERO
  clamping force through the pause/lift/carry (Bill's eyewitness,
  2026-07-05: 'closed on a graspable spot, then relaxed'). The
  tape-era lifts survived because adhesion needs no normal force;
  friction pads need it. Release via open (which de-energizes
  afterward as before)."
  [mm]
  (let [open-width 40
        target (/ mm open-width)
        pulses (int (+ close-pulse
                       (* target (- open-pulse close-pulse))))]
    (println "grip-hold pulses" (int pulses))
    (servo pulses)
    (Thread/sleep 2000)))

(comment
  (open)
  (close)
  (grip 30)


  )

