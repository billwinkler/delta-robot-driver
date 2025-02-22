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

;; First, define a function to get the neutral (raw) angle.
(defn neutral-theta []
  ;; Use your inverse function at the physical neutral z.
  ;; Here we assume (delta-calc-inverse 0 0 43) returns ~{:theta1 0.2256 ...}
  (:theta1 (delta-calc-inverse 0 0 43)))

;; Define the calibration factors based on your measurements.
(def k-retract 1.133)  ;; For raw angles >= 0
(def k-extend 1.255)   ;; For raw angles < 0; these factors can be tuned.

(defn correct-theta [theta-raw]
  "Corrects a raw theta value from delta-calc-inverse so that
   neutral becomes 0, retracted angles (raw >= 0) are scaled to match a maximum of about 25°,
   and extended angles (raw < 0) are scaled and remapped into the [0,360) range (with extended ~275°)."
  (let [neutral (neutral-theta)
        raw (- theta-raw neutral)]
    (if (>= raw 0)
      ;; For retracted angles, scale by k-retract.
      (* raw k-retract)
      ;; For extended angles (raw is negative), scale by k-extend and add 360.
      (mod (+ 360 (* raw k-extend)) 360))))

;; Now define a corrected inverse kinematics function:
(defn delta-calc-inverse-corrected
  "Inverse kinematics with corrected theta angles.
   Given the end-effector position (x, y, z) in physical coordinates,
   returns a map with keys :theta1, :theta2, :theta3 corresponding to the physical angles.
   Angles are measured as positive clockwise from the x-axis,
   with 0° at neutral, retracted values from 0° to ~25°, and extended values from ~275° to 360°."
  [x y z]
  (let [{theta1 :theta1 theta2 :theta2 theta3 :theta3 :as raw}
        (delta-calc-inverse x y z)]
    (if (or (nil? theta1) (nil? theta2) (nil? theta3))
      nil
      {:theta1 (correct-theta theta1)
       :theta2 (correct-theta theta2)
       :theta3 (correct-theta theta3)})))

;; Example usage:
(println "Neutral (z=43) corrected:" (delta-calc-inverse-corrected 0 0 43))
;; Neutral (z=43) corrected: {:theta1 0.0, :theta2 0.0, :theta3 0.0}
(println "Extended (z=110) corrected:" (delta-calc-inverse-corrected 0 0 110))
;; Extended (z=110) corrected: {:theta1 291.4158792839843, :theta2 291.4158792839843, :theta3 291.4158792839843}
(println "Extended (z=115) corrected:" (delta-calc-inverse-corrected 0 0 115))
;; Extended (z=115) corrected: {:theta1 263.0876639110111, :theta2 263.0876639110111, :theta3 263.0876639110111}


