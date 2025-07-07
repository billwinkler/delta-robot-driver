(ns delta-robot.limit-switch-test
  (:require [clojure.test :refer [deftest is testing]]
            [babashka.process :refer [sh]]
            [clojure.string :as str]
            [delta-robot.config :refer [config]]))

(defn- read-gpio [pin]
  (let [result (sh "pigs" "r" (str pin))]
    (-> result :out str/trim Integer/parseInt)))

#_(deftest test-limit-switches
  (testing "Limit switch functionality"
    (let [limit-switch-pins (get-in config [:gpio-pins :limit-switches])]
      (doseq [pin limit-switch-pins]
        (println (str "Testing limit switch on GPIO pin " pin "."))
        (println "Please press and hold the switch...")
        (Thread/sleep 2000) ; Give user time to press
        (is (= 0 (read-gpio pin)) (str "Limit switch on pin " pin " should be active (0)"))
        (println "Please release the switch...")
        (Thread/sleep 2000) ; Give user time to release
        (is (= 1 (read-gpio pin)) (str "Limit switch on pin " pin " should be inactive (1)"))))))
