(ns delta-robot.delta-geometry)

;; exploring code from https://hypertriangle.com/~alex/delta-robot-tutorial/

;; Constants for robot geometry
(def e 69.0)   ;; end effector triangle edge length
(def f 173.0)  ;; base triangle edge length
(def re 320.0) ;; forearm length
(def rf 130.0) ;; upper arm length

;; Trigonometric constants
(def sqrt3 (Math/sqrt 3.0))
(def pi Math/PI)
(def sin120 (/ sqrt3 2.0))
(def cos120 -0.5)
(def tan60 sqrt3)
(def sin30 0.5)
(def tan30 (/ 1 sqrt3))

;; Forward kinematics
(defn delta-calc-forward [theta1 theta2 theta3]
  "Forward kinematics, takes the three joint angles in 
   degrees and returns the effector x,y,z position"
  (let [t (* (/ tan30 2) (- f e))
        dtr (/ pi 180.0)
        ;; the joint angles
        theta1 (* theta1 dtr)
        theta2 (* theta2 dtr)
        theta3 (* theta3 dtr)
        y1 (- t (* rf (Math/cos theta1)))
        z1 (- (* -1 rf (Math/sin theta1)))
        y2 (* (+ t (* rf (Math/cos theta2))) sin30)
        x2 (* y2 tan60)
        z2 (- (* -1 rf (Math/sin theta2)))
        y3 (* (+ t (* rf (Math/cos theta3))) sin30)
        x3 (* -1 y3 tan60)
        z3 (- (* -1 rf (Math/sin theta3)))
        dnm (- (* (- y2 y1) x3) (* (- y3 y1) x2))
        w1 (+ (* y1 y1) (* z1 z1))
        w2 (+ (* x2 x2) (* y2 y2) (* z2 z2))
        w3 (+ (* x3 x3) (* y3 y3) (* z3 z3))
        a1 (- (* (- z2 z1) (- y3 y1)) (* (- z3 z1) (- y2 y1)))
        b1 (/ (- (* (- w2 w1) (- y3 y1)) (* (- w3 w1) (- y2 y1))) 2.0)
        a2 (- (* -1 (- z2 z1) x3) (* (- z3 z1) x2))
        b2 (/ (+ (* (- w2 w1) x3) (* (- w3 w1) x2)) 2.0)
        a (+ (* a1 a1) (* a2 a2) (* dnm dnm))
        b (* 2 (+ (* a1 b1) (* a2 (- b2 (* y1 dnm))) (* z1 (* dnm dnm))))
        c (+ (* (- b2 (* y1 dnm)) (- b2 (* y1 dnm)))
             (* b1 b1)
             (* dnm dnm (- (* z1 z1) (* re re))))
        d (- (* b b) (* 4 a c))]
    (if (neg? d)
      {:status -1}
      (let [z0 (/ (- (* -0.5 (+ b (Math/sqrt d))) a))
            x0 (/ (+ (* a1 z0) b1) dnm)
            y0 (/ (+ (* a2 z0) b2) dnm)]
        {:status 0 :x0 x0 :y0 y0 :z0 z0}))))

;; Inverse kinematics helper
(defn delta-calc-angle-yz [x0 y0 z0]
  (let [y1 (* -0.5 0.57735 f)
        y0 (- y0 (* 0.5 0.57735 e))
        a (/ (+ (* x0 x0) (* y0 y0) (* z0 z0) (* rf rf) (- (* re re)) (- (* y1 y1))) (* 2 z0))
        b (/ (- y1 y0) z0)
        d (+ (* -1 (+ (* a b) y1) (+ (* a b y1) (* rf (* b b rf))))
             (* rf rf))]
    (if (neg? d)
      {:status -1}
      (let [yj (/ (- y1 (* a b) (Math/sqrt d)) (+ (* b b) 1))
            zj (+ a (* b yj))
            theta (+ (* (/ 180 pi) (Math/atan (/ (- zj) (- y1 yj))))
                     (if (> yj y1) 180.0 0.0))]
        {:status 0 :theta theta}))))

;; Inverse kinematics
(defn delta-calc-inverse [x0 y0 z0]
  (let [res1 (delta-calc-angle-yz x0 y0 z0)
        res2 (delta-calc-angle-yz (+ (* x0 cos120) (* y0 sin120))
                                  (- (* y0 cos120) (* x0 sin120))
                                  z0)
        res3 (delta-calc-angle-yz (- (* x0 cos120) (* y0 sin120))
                                  (+ (* y0 cos120) (* x0 sin120))
                                  z0)]
    (if (and (= 0 (:status res1))
             (= 0 (:status res2))
             (= 0 (:status res3)))
      {:status 0 :theta1 (:theta res1) :theta2 (:theta res2) :theta3 (:theta res3)}
      {:status -1})))
