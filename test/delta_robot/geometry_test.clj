(ns delta-robot.geometry-test
  (:require [clojure.test :refer [deftest is testing]]
            [delta-robot.core :refer [delta-angle config]]))

(def pivot-x 34.64)
(def pivot-z 0.0)
(def upper-arm 50.0)
(def lower-arm 80.0)     
(def effector-radius 17.3205)   ; effective horizontal offset on the effector

;; A small helper for approximate floating comparison:
(defn approx=
  [x y tol]
  (<= (Math/abs (- x y)) tol))

;; Here’s an example table. 
;; In your Fusion data, you have an angle and a measured z, but typically 
;; you’d also need x (and possibly y=0 if you’re in a 2D plane). 
;; For demonstration, let’s store just the angles you tested in Fusion, 
;; and we’ll do the geometry in `forward-kinematics`.
(def test-data
  [{:theta 28}
   {:theta 25}
   {:theta 20}
   ;; ...
   {:theta 285}])

(defn place-arms [theta-deg]
  ;; 1) Upper-arm pivot -> elbow
  ;;   The pivot is at (34.64,0,0).
  ;;   The upper arm is 50 mm, oriented θ deg clockwise from +x.
  (let [theta-rad (Math/toRadians theta-deg)
        dx (* upper-arm (Math/cos theta-rad))     ; 2D in x
        dz (* upper-arm (- (Math/sin theta-rad))) ; negative for "clockwise" down
        elbow-x (+ pivot-x dx)
        elbow-z (+ 0 dz)]
    ;; 2) Elbow -> effector
    ;;   The lower arm is 80 mm, and from the diagram the difference is
    ;;     [-71.29, 36.29] in the XZ plane if θ=25
    ;;   But in general you might have a second angle or a constraint
    ;;   that sets the direction. For now, let's just hardcode the example
    (let [ex (- elbow-x 71.29)    ; x minus 71.29
          ez (+ elbow-z 36.29)]
      {:elbow    [elbow-x 0 elbow-z]
       :effector [ex 0 ez]})))


(defn forward-kinematics
  "Given a Fusion angle (θ in degrees, measured clockwise from the +x axis),
   compute a map with:
     - :elbow  as the [x y z] position of the elbow (end of the upper arm)
     - :effector as the [x y z] position of the effector joint, computed
       by treating the lower arm as a right triangle whose hypotenuse is 80,
       with base = (elbow‑x − effector‑radius), so that:
           vertical offset = sqrt(80² − (elbow‑x − effector‑radius)²)
       and the effector z is the elbow z plus that offset.
   
   Note: We convert the Fusion angle (which increases clockwise)
         by negating it when converting to radians."
  [theta-deg]
  (let [theta-rad (Math/toRadians (- theta-deg))
        ;; Compute elbow position from upper arm rotation.
        elbow-x (+ pivot-x (* upper-arm (Math/cos theta-rad)))
        elbow-y 0
        elbow-z (+ pivot-z (* upper-arm (Math/sin theta-rad)))
        ;; Compute horizontal distance (base of lower arm triangle)
        base (- elbow-x effector-radius)
        ;; Ensure we don’t take the square root of a negative number.
        base-sq (Math/pow base 2)
        hyp-sq (Math/pow lower-arm 2)
        vertical (if (<= base-sq hyp-sq)
                   (Math/sqrt (- hyp-sq base-sq))
                   0)
        effector-z (+ elbow-z vertical)]
    {:theta theta-deg
     :base base
     :vertical vertical
     :elbow [elbow-x elbow-y elbow-z]
     :effector [elbow-x elbow-y effector-z]}))

(deftest test-delta-angle-vs-fusion
  (testing "Compare code’s delta-angle to Fusion angles"
    (doseq [{:keys [theta]} test-data]
      (let [[x y z] (forward-kinematics theta)
            ;; Now call delta-angle for motor 0 with that (x,y,z)
            ;; to see what angle the IK code computes.
            computed-angle (delta-angle 0 x y z config)]
        (is (approx= computed-angle theta 1.0)
            (str "Angle mismatch. Fusion says " theta 
                 "°, code computed " computed-angle "°"))))))
