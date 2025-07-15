(ns test-motion
  (:require [delta-robot.motion :as move]))

;;(move/move-path [[0 0 400]])
(move/home)
(move/move-path [[20 0 400]])


