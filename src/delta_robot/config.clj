(ns delta-robot.config
  (:require [clojure.edn :as edn]))

(def config
  (-> (slurp "config.edn")
      edn/read-string
      (as-> cfg (assoc cfg :base-radius (/ (:base-edge cfg) (Math/sqrt 3)))
              (assoc cfg :effector-radius (* (/ (Math/sqrt 3) 3) (:effector-edge cfg))))))
