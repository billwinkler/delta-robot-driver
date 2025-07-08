(ns test-pigpio
  (:require [babashka.process :refer [sh]]
            [clojure.string :as str]
            [clojure.test :refer [deftest is testing run-tests]]))

(defn- run-pigs [args]
  (-> (apply sh "pigs" args)
      :out
      str/trim))

(deftest gpio-pin-test
  (testing "Read and write to GPIO pin 20"
    (println "Setting GPIO 20 mode to OUTPUT")
    (run-pigs ["m" "20" "w"])
    
    (println "Writing 1 to GPIO 20")
    (run-pigs ["w" "20" "1"])
    (is (= "1" (run-pigs ["r" "20"])) "Pin should be high (1)")
    
    (println "Writing 0 to GPIO 20")
    (run-pigs ["w" "20" "0"])
    (is (= "0" (run-pigs ["r" "20"])) "Pin should be low (0)")))

(run-tests 'test-pigpio)
