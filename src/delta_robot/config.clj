(ns delta-robot.config
  (:require [clojure.edn :as edn]))

;;(def config (edn/read-string (slurp "../../config.edn")))

(def config (edn/read-string (slurp "resources/config.edn")))

(defn motor-step-pins []
  (->> (:gpio-pins config)
       (vals)
       (filter map?) ;; exclude the :limit-switches vector
       (mapv :step)))

(defn motor-direction-pins []
  (->> (:gpio-pins config)
       (vals)
       (filter map?) ;; exclude the :limit-switches vector
       (mapv :dir)))

(defn limit-switch-pins []
  (get-in config [:gpio-pins :limit-switches]))


