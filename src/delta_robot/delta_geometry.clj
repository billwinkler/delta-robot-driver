(ns delta-robot.delta-geometry
  "Kinematics and calibration for a delta robot.
   Provides functions for forward and inverse kinematics, as well as calibration correction based on physical measurements.")

;; Global geometry constants
(def ^:const e 30.0)   ; end-effector equilateral triangle side
(def ^:const f 60.0)   ; base equilateral triangle side
(def ^:const rf 50.0)  ; upper arm length
(def ^:const re 80.0)  ; lower arm length
(def ^:const tan30 (Math/tan (/ Math/PI 6)))
(def ^:const deg-to-rad (/ Math/PI 180.0))
(def ^:const rad-to-deg (/ 180.0 Math/PI))

;; -------------------------------------------------
;; Forward Kinematics
;; -------------------------------------------------
(defn delta-calc-forward
  "Forward kinematics for a delta robot.
   Given three joint angles (in degrees), computes the effector position as a map {:x, :y, :z}.
   Returns nil if no valid solution exists."
  [theta1 theta2 theta3]
  (let [t1 (* theta1 deg-to-rad)
        t2 (* theta2 deg-to-rad)
        t3 (* theta3 deg-to-rad)
        ;; Trigonometric constants
        sin30 (Math/sin (/ Math/PI 6))
        tan60 (Math/tan (/ Math/PI 3))
        ;; Offset (common to each arm)
        t (* (/ (- f e) 2.0) tan30)
        ;; Calculate elbow joint positions for each arm:
        ;; Arm 1:
        y1 (- t (* rf (Math/cos t1)))
        z1 (- (* rf (Math/sin t1)))
        ;; Arm 2:
        y2 (* (+ t (* rf (Math/cos t2))) sin30)
        x2 (* y2 tan60)
        z2 (- (* rf (Math/sin t2)))
        ;; Arm 3:
        x3 (- (* (+ t (* rf (Math/cos t3))) sin30))
        y3 (* x3 tan60)
        z3 (- (* rf (Math/sin t3)))
        ;; Intermediate variables for sphere intersection:
        dnm (- (* (- y2 y1) x3) (* (- y3 y1) x2))
        w1 (+ (Math/pow y1 2) (Math/pow z1 2))
        w2 (+ (Math/pow x2 2) (Math/pow y2 2) (Math/pow z2 2))
        w3 (+ (Math/pow x3 2) (Math/pow y3 2) (Math/pow z3 2))
        a1 (- (* (- z2 z1) (- y3 y1))
              (* (- z3 z1) (- y2 y1)))
        a2 (- (* (- z3 z1) x2)
              (* (- z2 z1) x3))
        b1 (- (/ (- (* (- w2 w1) (- y3 y1))
                    (* (- w3 w1) (- y2 y1)))
                  (* 2 dnm)))
        b2 (/ (- (* (- w2 w1) x3)
                 (* (- w3 w1) x2))
              (* 2 dnm))
        a (+ (Math/pow a1 2) (Math/pow a2 2) (Math/pow dnm 2))
        b (* 2 (- (+ (* a1 b1) (* a2 b2))
                  (* (Math/pow dnm 2) z1)))
        c (+ (Math/pow b1 2) (Math/pow b2 2)
             (* (Math/pow dnm 2)
                (- (Math/pow z1 2) (Math/pow re 2))))
        d (- (Math/pow b 2) (* 4 a c))]
    (if (< d 0)
      nil
      (let [z0 (- (/ (+ b (Math/sqrt d)) (* 2 a)))
            x0 (/ (+ (* a1 z0) b1) dnm)
            y0 (/ (+ (* a2 z0) b2) dnm)]
        {:x x0 :y y0 :z z0}))))

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
   Each map has keys :z and :theta1 (measured joint angle in degrees)."
  [{:z 28.637 :theta1 25}
   {:z 30.490 :theta1 20}
   {:z 32.825 :theta1 15}
   {:z 35.699 :theta1 10}
   {:z 39.158 :theta1 5}
   {:z 43.220 :theta1 0}
   {:z 47.873 :theta1 355}
   {:z 53.064 :theta1 350}
   {:z 58.707 :theta1 345}
   {:z 64.692 :theta1 340}
   {:z 70.898 :theta1 335}
   {:z 77.202 :theta1 330}
   {:z 83.485 :theta1 325}
   {:z 89.639 :theta1 320}
   {:z 95.566 :theta1 315}
   {:z 101.181 :theta1 310}
   {:z 111.191 :theta1 300}
   {:z 115.469 :theta1 295}
   {:z 119.201 :theta1 290}
   {:z 122.352 :theta1 285}
   {:z 124.897 :theta1 280}
   {:z 126.817 :theta1 275}])

(defn make-calibration-data
  "Builds calibration data from the physical model.
   For each entry, computes the raw theta using delta-calc-inverse,
   unwraps both the physical and computed theta values,
   and computes the error (physical - computed)."
  [physical-model]
  (map (fn [{:keys [z theta1] :as m}]
         (if-let [inv (delta-calc-inverse 0 0 z)]
           (let [computed (unwrap-angle (:theta1 inv))
                 physical-unwrapped (unwrap-angle theta1)
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
(defn delta-calc-inverse-corrected
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
  (println "Forward kinematics (10, 20, 30):" (delta-calc-forward 10 20 30))
  (println "Inverse kinematics (0, 0, 43):" (delta-calc-inverse 0 0 43))
  (println "Corrected inverse kinematics (0, 0, 43):" (delta-calc-inverse-corrected 0 0 43))
  (doseq [datum calibration-data]
    (println datum))
  (println "Interpolated error at z=50:" (interpolate-error 50))

  (println "Corrected inverse kinematics (0, 0, 43):" ))
;; => nil;; => nil
  (delta-calc-inverse-corrected 0 0 43)
;; => {:theta1 0.26276665896420326,
;;     :theta2 0.26276665896420326,
;;     :theta3 0.26276665896420326}
(delta-calc-forward 0.26276665896420326 0.26276665896420326 0.26276665896420326)
;; => {:x 0.027378343008122252,
;;     :y -0.027378343008122252,
;;     :z -80.22929699467375}
