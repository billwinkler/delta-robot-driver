(ns delta-robot.delta-geometry
  "Kinematics and calibration for a delta robot.
   Provides functions for forward and inverse kinematics, as well as calibration correction based on physical measurements."
(:require [delta-robot.config :as cfg]
          [clojure.math :as m]))

(def PI m/PI)
(def deg-to-rad (/ PI 180.0))
(def rad-to-deg (/ 180.0 PI))

;; Robot parameters (in mm)
(def base-radius 60.0)      ;; Radius of the base hip-joints
(def effector-radius 30.0)  ;; Radius of the effector attachment points
(def upper-arm-length 50.0) ;; Length of the upper arm
(def lower-arm-length 80.0) ;; Length of the lower arm

(defn rotate-coordinates
  "Rotate coordinates [x, y] by angle phi (in degrees) around Z-axis."
  [x y phi]
  (let [phi-rad (* phi deg-to-rad)
        cos-phi (m/cos phi-rad)
        sin-phi (m/sin phi-rad)]
    [(- (* x cos-phi) (* y sin-phi))
     (+ (* x sin-phi) (* y cos-phi))]))

(defn delta-calc-angle-yz
  "Compute the upper arm angle phi (in degrees) for one arm in the YZ plane."
  [x0 y0 z0]
  (let [y1 (- base-radius) ;; y-position of base pivot
        y0-adjusted (- y0 effector-radius) ;; adjust for effector offset
        ;; Coefficients for the quadratic equation derived from kinematics
        a (+ (* x0 x0) (* y0-adjusted y0-adjusted) (* z0 z0)
             (* upper-arm-length upper-arm-length)
             (- (* lower-arm-length lower-arm-length))
             (* y1 y1))
        a (/ a (* 2.0 z0))
        b (/ (- y1 y0-adjusted) z0)
        d (- (* upper-arm-length upper-arm-length (+ 1.0 (* b b)))
             (* (+ a (* b y1)) (+ a (* b y1))))
        sqrt-d (m/sqrt d)]
    (when (>= d 0)
      (let [yj (/ (- (- y1 (* a b)) sqrt-d) (+ 1.0 (* b b)))
            zj (+ a (* b yj))
            phi (m/atan2 zj (- y1 yj))]
        (* phi rad-to-deg)))))

(defn delta-calc-inverse
  "Compute theta angles for all three arms given effector position [x, y, z].
   Returns a map with :theta1, :theta2, :theta3, or nil if unreachable."
  [x y z]
  (let [angles (for [i [0 120 240]] ;; Angles for each arm
                 (let [[x-rot y-rot] (rotate-coordinates x y i)
                       phi (delta-calc-angle-yz x-rot y-rot z)]
                   (when phi
                     (let [theta (- 360.0 phi)] ;; Convert phi to theta
                       (if (> theta 360) (- theta 360) theta)))))]
    (when (every? some? angles)
      {:theta1 (first angles)
       :theta2 (second angles)
       :theta3 (last angles)})))

;; Example usage
(defn example []
  (let [positions [[0.0 0.0 101.396] ;; From diagram (theta = 300°)
                   [0.0 0.0 5.843]]  ;; From query (theta = 25°)
        results (map (fn [[x y z]]
                       (delta-calc-inverse x y z))
                     positions)]
    (println "Theta angles for positions:")
    (doseq [[pos angles] (zipmap positions results)]
      (println "Position" pos "->" angles))))


(def physical-model
  "A vector of maps representing the physical calibration model.
   Each map has keys :z and :theta (measured joint angle in degrees).
   All hip joints are set to the same angle theta"
  [{:z 5.843 :theta 25}
   {:z 4.656 :theta 20}
   {:z 3.481 :theta 15}
   {:z 2.316 :theta 10}
   {:z 1.156 :theta 5}
   {:z 0.000 :theta 0}
   {:z 9.872 :theta 355}
   {:z 19.681 :theta 350}
   {:z 29.363 :theta 345}
   {:z 38.858 :theta 340}
   {:z 48.105 :theta 335}
   {:z 57.046 :theta 330}
   {:z 65.625 :theta 325}
   {:z 73.790 :theta 320}
   {:z 81.493 :theta 315}
   {:z 88.688 :theta 310}
   {:z 95.334 :theta 300}
   {:z 101.396 :theta 295}
   {:z 111.649 :theta 290}
   {:z 115.795 :theta 285}
   {:z 119.267 :theta 280}
   {:z 122.056 :theta 275}])






