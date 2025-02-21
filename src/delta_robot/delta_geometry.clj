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
