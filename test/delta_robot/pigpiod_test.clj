(ns delta-robot.pigpiod-test
  (:require [clojure.test :refer [deftest is testing]]
            [babashka.process :refer [sh]]
            [clojure.string :as str]))

(deftest test-pigpiod-daemon
  (testing "pigpiod daemon is running"
    (let [result (sh "pgrep" "pigpiod")]
      (is (zero? (:exit result)) "pigpiod process should be running")
      (is (not (str/blank? (:out result))) "pgrep should return a PID"))))
