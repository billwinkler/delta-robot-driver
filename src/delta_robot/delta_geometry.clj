(ns delta-robot.delta-geometry
  "Kinematics and calibration for a delta robot.
   Provides functions for forward and inverse kinematics, as well as calibration correction based on physical measurements."
(:require [delta-robot.config :as cfg]))

;; Global geometry constants
(def ^:const e (:effector-edge cfg/config))   ; end-effector equilateral triangle side
(def ^:const f (:base-edge cfg/config))       ; base equilateral triangle side
(def ^:const rf (:upper-arm-length cfg/config))  ; upper arm length
(def ^:const re (:lower-arm-length cfg/config))  ; lower arm length
(def ^:const tan30 (Math/tan (/ Math/PI 6)))
(def ^:const deg-to-rad (/ Math/PI 180.0))
(def ^:const rad-to-deg (/ 180.0 Math/PI))
(def ^:const tan60 (Math/tan (/ Math/PI 3)))
(def ^:const z-offset 11.2)  ; same offset as used in delta-calc-inverse

;; -------------------------------------------------
;; Inverse Kinematics
;; -------------------------------------------------
(defn delta-calc-angle-yz
  "Computes the joint angle (in degrees) for one arm given the
   transformed coordinates (x0, y0, z0). Returns nil if no valid solution exists."
  [x0 y0 z0]
  (let [y1 (* -0.5 tan30 f)
        y0-adjusted (- y0 (* 0.5 tan30 e))
        a (/ (+ (Math/pow x0 2)
                (Math/pow y0-adjusted 2)
                (Math/pow z0 2)
                (Math/pow rf 2)
                (- (Math/pow re 2))
                (Math/pow y1 2))
             (* 2 z0))
        b (/ (- y1 y0-adjusted) z0)
        d (- (* rf rf (+ 1 (* b b)))
             (Math/pow (+ a (* b y1)) 2))]
    (if (< d 0)
      nil
      (let [yj (/ (- y1 (* a b) (Math/sqrt d))
                  (+ 1 (* b b)))
            zj (+ a (* b yj))
            theta (Math/atan2 (- zj) (- y1 yj))]
        (* rad-to-deg theta)))))

(defn delta-calc-inverse
  "Inverse kinematics for a delta robot.
   Given the end-effector position (x, y, z) in physical coordinates,
   returns a map with keys :theta1, :theta2, and :theta3 (in degrees) corresponding to the required joint angles.
   Returns nil if any arm has no valid solution."
  [x y z]
  (let [z-offset 11.2                       ; offset to adjust z coordinate (calibration)
        z-adjusted (+ z z-offset)
        ;; For arm 1 (no rotation)
        theta1 (delta-calc-angle-yz x y z-adjusted)
        ;; For arm 2, rotate coordinates by +120°
        angle120 (/ (* 2 Math/PI) 3)
        cos120 (Math/cos angle120)
        sin120 (Math/sin angle120)
        x2 (+ (* x cos120) (* y sin120))
        y2 (- (* y cos120) (* x sin120))
        theta2 (delta-calc-angle-yz x2 y2 z-adjusted)
        ;; For arm 3, rotate coordinates by -120°
        x3 (+ (* x (Math/cos (- angle120))) (* y (Math/sin (- angle120))))
        y3 (- (* y (Math/cos (- angle120))) (* x (Math/sin (- angle120))))
        theta3 (delta-calc-angle-yz x3 y3 z-adjusted)]
    (if (or (nil? theta1) (nil? theta2) (nil? theta3))
      nil
      {:theta1 theta1 :theta2 theta2 :theta3 theta3})))

;; -------------------------------------------------
;; Calibration Utilities
;; -------------------------------------------------
(defn unwrap-angle
  "Converts an angle from the 0–360° range into a range centered around 0.
   For example, 355 becomes -5 and 350 becomes -10."
  [theta]
  (if (> theta 180)
    (- theta 360)
    theta))

(def physical-model
  "A vector of maps representing the physical calibration model.
   Each map has keys :z and :theta (measured joint angle in degrees).
   All hip joints are set to the same angle theta"
  [{:z 28.637 :theta 25}
   {:z 30.490 :theta 20}
   {:z 32.825 :theta 15}
   {:z 35.699 :theta 10}
   {:z 39.158 :theta 5}
   {:z 43.220 :theta 0}
   {:z 47.873 :theta 355}
   {:z 53.064 :theta 350}
   {:z 58.707 :theta 345}
   {:z 64.692 :theta 340}
   {:z 70.898 :theta 335}
   {:z 77.202 :theta 330}
   {:z 83.485 :theta 325}
   {:z 89.639 :theta 320}
   {:z 95.566 :theta 315}
   {:z 101.181 :theta 310}
   {:z 111.191 :theta 300}
   {:z 115.469 :theta 295}
   {:z 119.201 :theta 290}
   {:z 122.352 :theta 285}
   {:z 124.897 :theta 280}
   {:z 126.817 :theta 275}])

(defn make-calibration-data
  "Builds calibration data from the physical model.
   For each entry, computes the raw theta using delta-calc-inverse,
   unwraps both the physical and computed theta values,
   and computes the error (physical - computed)."
  [physical-model]
  (map (fn [{:keys [z theta] :as m}]
         (if-let [inv (delta-calc-inverse 0 0 z)]
           (let [computed (unwrap-angle (:theta1 inv))
                 physical-unwrapped (unwrap-angle theta)
                 err (- physical-unwrapped computed)]
             {:z z
              :physical physical-unwrapped
              :computed computed
              :err err})
           (assoc m :computed nil :err nil)))
       physical-model))

(def calibration-data
  "Calibration data computed from the physical model."
  (vec (make-calibration-data physical-model)))

(defn interpolate-error
  "Linearly interpolates an error correction for a given z value using calibration-data.
   Returns nil if z is outside the calibration range."
  [z]
  (let [data calibration-data]
    (if (or (< z (:z (first data)))
            (> z (:z (last data))))
      nil
      (loop [pairs (partition 2 1 data)]
        (if (empty? pairs)
          nil
          (let [[p q] (first pairs)]
            (if (and (<= (:z p) z) (>= (:z q) z))
              (let [t (/ (- z (:z p))
                         (- (:z q) (:z p)))
                    err-p (:err p)
                    err-q (:err q)]
                (+ err-p (* t (- err-q err-p))))
              (recur (rest pairs)))))))))

;; -------------------------------------------------
;; Corrected Inverse Kinematics
;; -------------------------------------------------
(defn calibrated-delta-calc-inverse
  "Computes inverse kinematics and applies a calibration correction.
   It calls delta-calc-inverse to get raw joint angles, unwraps them,
   then adds an interpolated error (based on the current z) to produce corrected angles.
   Returns a map with keys :theta1, :theta2, and :theta3 (in degrees), or nil if no solution exists."
  [x y z]
  (if-let [{raw-theta1 :theta1 raw-theta2 :theta2 raw-theta3 :theta3}
           (delta-calc-inverse x y z)]
    (let [E (or (interpolate-error z) 0)
          corr-theta1 (+ (unwrap-angle raw-theta1) E)
          corr-theta2 (+ (unwrap-angle raw-theta2) E)
          corr-theta3 (+ (unwrap-angle raw-theta3) E)]
      {:theta1 corr-theta1
       :theta2 corr-theta2
       :theta3 corr-theta3})
    nil))


;; -------------------------------------------------
;; Example Usage (for testing, can be commented out)
;; -------------------------------------------------
(comment
  (println "Inverse kinematics (0, 0, 43):" (delta-calc-inverse 0 0 43))

  (println "Corrected inverse kinematics (0, 0, 43):" (calibrated-delta-calc-inverse 0 0 43))
  (doseq [datum calibration-data]
    (println datum))
  (println "Interpolated error at z=50:" (interpolate-error 50))

  (for [z (range 25 130 10)]
    (println [z (:theta1 (delta-calc-inverse 0 0 z))]))

  (unwrap-angle -8)
  )

(doseq [z (range 25 130 10)]
    (println [z (:theta1 (calibrated-delta-calc-inverse 0 0 z))]))
;; => (nil nil nil nil nil nil nil nil nil nil nil)
;; => ([30 9.403804171774706]
;;     [40 -2.9257792577574144]
;;     [50 -12.88434560184749]
;;     [60 -21.83631338562363]
;;     [70 -30.449319334131328]
;;     [80 -39.19589468448646]
;;     [90 -48.5846196331177]
;;     [100 -59.48803024931988]
;;     [110 -74.73589768335175]
;;     [120 nil]
;;     [130 nil])




