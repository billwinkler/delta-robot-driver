(ns delta-robot.delta-geometry)

;; exploring code from https://hypertriangle.com/~alex/delta-robot-tutorial/
(defn delta-calc-forward [theta1 theta2 theta3]
  "Forward kinematics for a delta robot.
   Takes three joint angles (in degrees) and returns a map with the effector x, y, z position.
   Uses parameters:
     e  - end-effector equilateral triangle side,
     f  - base equilateral triangle side,
     rf - upper arm length,
     re - lower arm length.
   Returns nil if no valid solution exists."
  (let [pi Math/PI
        deg-to-rad (/ pi 180.0)
        ;; Convert angles from degrees to radians.
        t1 (* theta1 deg-to-rad)
        t2 (* theta2 deg-to-rad)
        t3 (* theta3 deg-to-rad)
        ;; Robot geometry constants (example values – adjust as needed)
        e 30
        f 60
        rf 50
        re 80
        ;; Trigonometric constants
        sin30 (Math/sin (/ pi 6))   ; 0.5
        tan30 (Math/tan (/ pi 6))   ; ≈0.57735
        tan60 (Math/tan (/ pi 3))   ; ≈1.73205
        ;; Offset (common to each arm)
        t (* (/ (- f e) 2.0) tan30)
        ;; Calculate the positions of the three elbow joints:
        ;; For arm 1:
        y1 (- t (* rf (Math/cos t1)))   ; y1 = -(t + rf*cos(t1))
        z1 (- (* rf (Math/sin t1)))       ; z1 = -rf*sin(t1)
        ;; For arm 2:
        y2 (* (+ t (* rf (Math/cos t2))) sin30)
        x2 (* y2 tan60)
        z2 (- (* rf (Math/sin t2)))
        ;; For arm 3:
        x3 (- (* (+ t (* rf (Math/cos t3))) sin30))
        y3 (* x3 tan60)
        z3 (- (* rf (Math/sin t3)))
        ;; Intermediate variables used in the sphere intersection:
        dnm (- (* (- y2 y1) x3) (* (- y3 y1) x2))
        w1 (+ (Math/pow y1 2) (Math/pow z1 2))
        w2 (+ (Math/pow x2 2) (Math/pow y2 2) (Math/pow z2 2))
        w3 (+ (Math/pow x3 2) (Math/pow y3 2) (Math/pow z3 2))
        a1 (- (* (- z2 z1) (- y3 y1))
               (* (- z3 z1) (- y2 y1)))
        a2 (- (* (- z3 z1) x2)
               (* (- z2 z1) x3))
        ;; b1 and b2 are defined as:
        b1 (- (/ (- (* (- w2 w1) (- y3 y1))
                    (* (- w3 w1) (- y2 y1)))
                  (* 2 dnm)))
        b2 (/ (- (* (- w2 w1) x3)
                 (* (- w3 w1) x2))
              (* 2 dnm))
        ;; Coefficients for the quadratic equation in z:
        a (+ (Math/pow a1 2) (Math/pow a2 2) (Math/pow dnm 2))
        b (* 2 (- (+ (* a1 b1) (* a2 b2))
                  (* (Math/pow dnm 2) z1)))
        c (+ (Math/pow b1 2) (Math/pow b2 2)
             (* (Math/pow dnm 2)
                (- (Math/pow z1 2) (Math/pow re 2))))
        d (- (Math/pow b 2) (* 4 a c))]
    (if (< d 0)
      ;; no solution exists for the given angles
      nil
      (let [z0 (- (/ (+ b (Math/sqrt d)) (* 2 a)))
            x0 (/ (+ (* a1 z0) b1) dnm)
            y0 (/ (+ (* a2 z0) b2) dnm)]
        {:x x0 :y y0 :z z0}))))

;; Example usage:
;; (delta-calc-forward 10 20 30)


;; Define geometry constants once at the top of your namespace
(def ^:const e 30.0)
(def ^:const f 60.0)
(def ^:const rf 50.0)
(def ^:const re 80.0)
(def ^:const tan30 (Math/tan (/ Math/PI 6)))

(defn delta-calc-angle-yz
  "Computes the joint angle (in degrees) for one arm given the
   transformed coordinates (x0, y0, z0). Returns nil if no valid solution exists."
  [x0 y0 z0]
  (let [y1 (* -0.5 tan30 f)
        ;; Adjust y coordinate relative to the end effector.
        y0 (- y0 (* 0.5 tan30 e))
        a (/ (+ (* x0 x0)
                (* y0 y0)
                (* z0 z0)
                (* rf rf)
                (- (* re re))
                (- (* y1 y1)))
             (* 2 z0))
        b (/ (- y1 y0) z0)
        d (- (* rf rf (+ 1 (* b b)))
             (Math/pow (+ a (* b y1)) 2))]
    (if (< d 0)
      nil
      (let [yj (/ (- y1 (* a b) (Math/sqrt d))
                  (+ 1 (* b b)))
            zj (+ a (* b yj))
            theta (Math/atan2 (- zj) (- y1 yj))]
        (* 180.0 (/ theta Math/PI))))))
(defn delta-calc-inverse
  "Inverse kinematics for a delta robot.
   Given the end-effector position (x, y, z) in physical coordinates,
   returns a map with keys :theta1, :theta2, and :theta3 (in degrees)
   corresponding to the required joint angles.
   Adjusts the z coordinate by an offset so that physical z=43.2 gives theta=0."
  [x y z]
  (let [;; The offset needed: our calculations assume z=54.4 for theta=0,
        ;; but physically you have z=43.2. So we need to add 11.2.
        z-offset 11.2
        z-adjusted (+ z z-offset)
        pi Math/PI
        tan30 (Math/tan (/ pi 6))
        ;; Robot geometry constants (must be consistent across your functions)
        e 30.0
        f 60.0
        rf 50.0
        re 80.0
        ;; For arm 1 (no rotation)
        theta1 (delta-calc-angle-yz x y z-adjusted)
        ;; For arm 2, rotate coordinates by +120°
        angle120 (/ (* 2 pi) 3)
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

(def neutral-theta (:theta1 (delta-calc-inverse 0 0 43)))
;; => roughly 0.2256

(defn remove-neutral
  "Then define a corrected angle function that subtracts that neutral offse"
  [theta]
  (- theta neutral-theta))

;; At physical z corresponding to full retraction, my computed angle is about 22.2937°
;; (after removal of neutral, that’s 22.2937 – 0.2256 ≈ 22.0681°) and I want –25°.
;; This suggests a linear mapping:

;;   physical-θ = A × (θ_raw − neutral-theta) + 0
;;   and we want A × 22.0681 ≈ –25.
(def A (/ -25 22.0681))  ; Approximately -1.133

;; Then define a correction function
(defn correct-theta [theta-raw]
  (let [neutral-theta (:theta1 (delta-calc-inverse 0 0 43))
        A (/ -25 (- (:theta1 (delta-calc-inverse 0 0 28)) neutral-theta))]
    (+ (* A (- theta-raw neutral-theta)) 0)))  ; 0 here is the desired neutral physical angle.

(defn delta-calc-inverse-corrected
  "Inverse kinematics with corrected theta angles.
   Given (x y z) in physical coordinates, returns a map with corrected
   :theta1, :theta2, and :theta3 that are calibrated to your physical geometry."
  [x y z]
  (let [{theta1 :theta1 theta2 :theta2 theta3 :theta3 :as raw} (delta-calc-inverse x y z)]
    (if (or (nil? theta1) (nil? theta2) (nil? theta3))
      nil
      {:theta1 (correct-theta theta1)
       :theta2 (correct-theta theta2)
       :theta3 (correct-theta theta3)})))


(delta-calc-inverse 0 0 -79)
;; => {:theta1 23.954089748518864,
;;     :theta2 23.954089748518864,
;;     :theta3 23.954089748518864}
 (delta-calc-forward 23.95 23.95 23.95)
;; => {:x 0.03628612751807013,
;;     :y -0.03628612751807013,
;;     :z -100.29694701636205}

(delta-calc-inverse 0 0 0)
;; => java.lang.ArithmeticException: Divide by zero delta-robot.delta-geometry /Users/billwinkler/dev/delta-arm-v2/babashka/src/delta_robot/delta_geometry.clj:8:11

;; for the physical arm geometry, when theta=0, z=43.2
;; but the calculated inverse show theta=0 with z=54.4
(delta-calc-inverse 0 0 54.396)
;; => {:theta1 5.243397184967078E-4,
;;     :theta2 5.243397184967078E-4,
;;     :theta3 5.243397184967078E-4};; => nil
(for [z (range 54.39 54.40 0.001)]
  (assoc (delta-calc-inverse 0 0 z) :z z))

;; neutral point
(delta-calc-inverse 0 0 43)
;; => {:theta1 0.2256033455493592,
;;     :theta2 0.2256033455493592,
;;     :theta3 0.2256033455493592}
;; fully retracted, physical z~=18, and θ=-25
(delta-calc-inverse 0 0 28)
;; => {:theta1 22.293689663487974,
;;     :theta2 22.293689663487974,
;;     :theta3 22.293689663487974}
;; fully extended, physical z~=127 and θ=85
(delta-calc-inverse 0 0 127)
;; => nil
(for [z (range 100 130 10)]
  (assoc (delta-calc-inverse 0 0 z) :z z))
;; => ({:theta1 -54.42309842020025,
;;      :theta2 -54.42309842020025,
;;      :theta3 -54.42309842020025,
;;      :z 100}
;;     {:theta1 -67.48350197229738,
;;      :theta2 -67.48350197229738,
;;      :theta3 -67.48350197229738,
;;      :z 110}
;;     {:z 120})

(delta-calc-inverse-corrected 0 0 43)
;; => {:theta1 0.0, :theta2 0.0, :theta3 0.0}

;; fully retracted, physical-z~=18, and θ=-25
(delta-calc-inverse-corrected 0 0 28)
;; => {:theta1 -25.0, :theta2 -25.0, :theta3 -25.0}

;; when extended to 110, pysical-z~=110, then physical θ~=-59
(delta-calc-inverse-corrected 0 0 110)
;; => {:theta1 76.70477668787215,
;;     :theta2 76.70477668787215,
;;     :theta3 76.70477668787215}
