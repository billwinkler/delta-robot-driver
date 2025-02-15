(ns test-helper
  (:require [clojure.test :as t]))

(defmethod t/report [:fail] [m]
  (println "\nFAIL:" (:message m))
  (println "Expected:" (:expected m))
  (println "Got:" (:actual m)))

(defmethod t/report [:error] [m]
  (println "\nERROR:" (:message m))
  (println "Exception:" (:actual m)))

(defmethod t/report [:pass] [m]
  (println "PASS:" (:expected m)))
