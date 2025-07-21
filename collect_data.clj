(ns collect-data
  (:require [clojure.java.shell :as shell]
            [clojure.edn :as edn]
            [clojure.string :as str]
            [delta-robot.motion :as motion]))

(def config
  "Configuration for data collection."
  {:home-position [0 0 275]    ; The [x y z] coordinates over the pecan
   :z-height 275               ; Constant Z height for all movements
   :max-offset 50              ; Max random distance in mm for x and y
   :num-samples 10             ; Number of images to collect
   :data-dir "pecan_training_data"
   :images-dir "pecan_training_data/images"})

(defn setup-directories!
  "Create the necessary directories for storing data."
  []
  (shell/sh "mkdir" "-p" (:images-dir config)))

(defn random-offset
  "Generates a random [x y] offset."
  [max-val]
  [(- (rand-int (* 2 max-val)) max-val)
   (- (rand-int (* 2 max-val)) max-val)])

(defn capture-image!
  "Calls the python script to capture an image."
  [image-path]
  (let [{:keys [exit err]} (shell/sh "python" "python/camera.py" image-path)]
    (when-not (zero? exit)
      (println "Error capturing image:" err))))

(defn collect-samples
  "Main loop to collect training data."
  []
  (let [[home-x home-y home-z] (:home-position config)]
    (loop [i 0
           labels []]
      (if (< i (:num-samples config))
        (let [offset (random-offset (:max-offset config))
              [offset-x offset-y] offset
              target-x (+ home-x offset-x)
              target-y (+ home-y offset-y)
              image-name (format "sample_%04d.jpg" i)
              image-path (str (:images-dir config) "/" image-name)]

          (println (format "Sample %d/%d: offset %s" (inc i) (:num-samples config) offset))

          ; 1. Move to random offset position
          (motion/move-to target-x target-y (:z-height config))

          ; 2. Capture image
          (capture-image! image-path)

          ; 3. Move back to home
          (motion/move-to home-x home-y home-z)

          ; 4. Store label
          (recur (inc i) (conj labels {:image image-name :offset offset})))

        ;; Return all collected labels
        labels))))

(defn -main
  "Main entry point for the script."
  []
  (println "Starting data collection...")
  (setup-directories!)
  (motion/home) ; Home the robot before starting

  (let [collected-labels (collect-samples)
        labels-path (str (:data-dir config) "/labels.edn")]
    (println "\nSaving labels to" labels-path)
    (spit labels-path (with-out-str (clojure.pprint/pprint collected-labels))))

  (println "Data collection complete."))

;; To run this script:
;; bb -cp src -f collect_data.clj
(comment
  (-main))