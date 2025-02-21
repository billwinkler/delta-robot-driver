(ns inverse-kinematics
  (:require [clojure.edn :as edn]
            [babashka.fs :as fs]))

(comment
  ;; how to find the current working directory
  (require '[babashka.process :refer [shell]])
  (:out (shell {:out :string} "pwd")))

(def config
  (-> (slurp "babashka/config.edn")
      edn/read-string
      (as-> cfg (assoc cfg :base-radius (/ (:base-edge cfg) (Math/sqrt 3)))
        (assoc cfg :effector-radius (* (/ (Math/sqrt 3) 3) (:effector-edge cfg))))))

(keys config)
;; => (:upper-arm-length
;;     :lower-arm-length
;;     :base-edge
;;     :effector-edge
;;     :steps-per-rev
;;     :base-radius
;;     :effector-radius)

(run! (fn [[k v]] (println k ":" v)) config)


(defn degrees [rad]
  (* rad (/ 180 Math/PI)))

(defn calculate-motor-pulses [x y z]
  "Calculate motor pulses using the inverse kinematics for a delta robot.
   Returns a vector of motor pulses for each of the 3 motors."
  (let [R_b (:base-radius config)
        R_e (:effector-radius config)   
        L_a (:upper-arm-length config)
        L_s (:lower-arm-length config)
        steps-per-rev (:steps-per-rev config)
        attachment-angles [0 (/ Math/PI 3) (* 2 (/ Math/PI 3))]] ; 120-degree offsets
    (mapv (fn [angle]
            (let [x_base (+ (* R_b (Math/cos angle)))
                  y_base (+ (* R_b (Math/sin angle)))
                  distance (Math/sqrt
                            (+ (Math/pow (- x x_base) 2)
                               (Math/pow (- y y_base) 2)
                               (Math/pow (- z L_a) 2)))]
              ;; Convert distance to motor steps
              (int (* steps-per-rev (/ (- distance L_a) (* Math/PI L_a))))))
          attachment-angles)))

(calculate-motor-pulses 10 10 10)
;; => [-40 -95 49]

(calculate-motor-pulses 0 0 0)
;; => [220 220 220]
