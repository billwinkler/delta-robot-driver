(ns delta-robot.config
  (:require [clojure.edn :as edn]))

(def config (edn/read-string (slurp "config.edn")))
