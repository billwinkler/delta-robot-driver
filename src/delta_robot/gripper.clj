(ns delta-robot.gripper
  (:require [delta-robot.command-driver :refer [execute-pigs-cmd]]
            [delta-robot.config :refer [effector-servo-pin]]))

(defn- servo
  [pulse-width]
  (execute-pigs-cmd "servo" (effector-servo-pin) pulse-width))

(comment

  )

