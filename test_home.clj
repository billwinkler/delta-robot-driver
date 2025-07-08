(ns test-home
  (:require [delta-robot.command-driver :as driver]))

(driver/send-debug-pulses 1 1)
