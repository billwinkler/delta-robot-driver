(ns delta-robot.state
  (:require [clojure.java.io :as io]
            [clojure.edn :as edn]
            [clojure.tools.logging :as log]))

(def state-file (io/file "resources/state.edn"))

(defn save-state [state-map]
  (log/info "Saving state to" (.getAbsolutePath state-file))
  (spit state-file (pr-str state-map)))

(defn load-state []
  (if (.exists state-file)
    (try
      (let [state (edn/read-string (slurp state-file))]
        (log/info "Loading state from" (.getAbsolutePath state-file) ":" state)
        state)
      (catch Exception e
        (log/error "Error loading state, using default. Error:" (.getMessage e))
        nil))
    (do
      (log/info "State file not found, using default state.")
      nil)))
