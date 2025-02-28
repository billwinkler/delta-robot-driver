(ns delta-robot.inverse-kinematics
  (:require [clojure.math :as m]
            [delta-robot.config :as cfg]))

;; Constants and Helper Functions
(def PI Math/PI)
(def deg-to-rad (/ PI 180.0))
(def rad-to-deg (/ 180.0 PI))
(def phi-angles (map #(Math/toRadians (* % 120.0)) [0 1 2])) ; [0°, 120°, 240° in radians]

;; Compute hip joint position F_i for given phi
(defn compute-F-i [phi]
  [(* (:base-radius cfg/config) (Math/cos phi))
   (* (:base-radius cfg/config) (Math/sin phi))
   0.0])

;; Compute effector attachment point E_i for given [x, y, z] and phi
(defn compute-E-i [x y z phi]
  [(+ x (* (:effector-radius cfg/config) (Math/cos phi)))
   (+ y (* (:effector-radius cfg/config) (Math/sin phi)))
   z])

;; Compute elbow joint position J_i for given alpha and phi
;; J_i = F_i + upper-arm-length * [cos(alpha) * cos(phi), cos(alpha) * sin(phi), sin(alpha)]
(defn compute-J-i [alpha phi]
  (let [F-i (compute-F-i phi)
        upper-arm-length (:upper-arm-length cfg/config)
        delta-J [(* upper-arm-length (Math/cos alpha) (Math/cos phi))
                 (* upper-arm-length (Math/cos alpha) (Math/sin phi))
                 (* upper-arm-length (Math/sin alpha))]]
    (map + F-i delta-J)))

;; Compute derivative of J_i with respect to alpha
;; dJ_i/dalpha = upper-arm-length * [-sin(alpha) * cos(phi), -sin(alpha) * sin(phi), cos(alpha)]
(defn compute-dJ-i-dalpha [alpha phi]
  (let [upper-arm-length (:upper-arm-length cfg/config)]
    [(* upper-arm-length (- (Math/sin alpha)) (Math/cos phi))
     (* upper-arm-length (- (Math/sin alpha)) (Math/sin phi))
     (* upper-arm-length (Math/cos alpha))]))

;; Compute residual function f(alpha) = ||J_i(alpha) - E_i||^2 - lower-arm-length^2
(defn compute-f [alpha phi x y z]
  (let [J-i (compute-J-i alpha phi)
        E-i (compute-E-i x y z phi)
        diff (map - J-i E-i)
        dist-sq (reduce + (map #(* % %) diff))
        lower-arm-length (:lower-arm-length cfg/config)]
    (- dist-sq (* lower-arm-length lower-arm-length))))

;; Compute derivative of f(alpha)
;; f'(alpha) = 2 * (J_i - E_i) · dJ_i/dalpha
(defn compute-f-prime [alpha phi x y z]
  (let [J-i (compute-J-i alpha phi)
        E-i (compute-E-i x y z phi)
        diff (map - J-i E-i)
        dJ-dalpha (compute-dJ-i-dalpha alpha phi)
        dot-prod (reduce + (map * diff dJ-dalpha))]
    (* 2.0 dot-prod)))

;; Newton's method to solve for alpha
(def tolerance 1e-6)
(def max-iterations 1000)

(defn newtons-method [phi x y z]
  (loop [alpha 0.5 ; initial guess: 0.5 radians (~28.65°)
         iter 0]
    (if (>= iter max-iterations)
      nil ; failed to converge
      (let [f (compute-f alpha phi x y z)
            f-prime (compute-f-prime alpha phi x y z)]
        (if (< (Math/abs f) tolerance)
          alpha
          (if (zero? f-prime)
            nil ; avoid division by zero
            (recur (- alpha (/ f f-prime)) (inc iter))))))))

;; Map alpha (in radians) to theta (in degrees)
;; theta = (360 - alpha_deg) mod 360
(defn compute-theta [alpha]
  (let [alpha-deg (* alpha rad-to-deg)
        theta-deg (- 360.0 alpha-deg)]
    (mod alpha-deg 360.0)))

;; Unwrap angles greater than 180, to become negative degrees from x-axis
(defn unwrap [angle]
  (if (< angle 180) (- angle) (- 360 angle)))


;; Compute theta angles for all three arms
(defn delta-calc-inverse [x y z]
  (let [alphas (map (fn [phi] (newtons-method phi x y z)) phi-angles)]
    (if (some nil? alphas)
      nil                    ; return nil if any arm fails to converge
      (let [thetas (map (comp unwrap compute-theta) alphas)]
        {:theta1 (first thetas)
         :theta2 (second thetas)
         :theta3 (nth thetas 2)}))))


(comment
  ;; these are measurements taken from the physical robot
  (let [in->mm #(* % 25.4)
        physical [{:retracted {:theta 28 :z (in->mm 8)}
                   :extended {:theta -90 :z (in->mm 17.32)}
                   :horizontal {:theta 0 :z (in->mm 10.625)}}]]
    physical)

  ;; these are measurement taken from Fusion 360 model
  (let [physical-model [{:z 217.098 :theta 25}
                        {:z 224.689 :theta 20}
                        {:z 233.166  :theta 15}
                        {:z 242.514 :theta 10}
                        {:z 252.701 :theta 5}
                        {:z 263.500 :theta 0}
                        {:z 275.362 :theta 355}
                        {:z 287.662 :theta 350}
                        {:z 300.485 :theta 345}
                        {:z 313.614 :theta 340}
                        {:z 326.979 :theta 335}
                        {:z 340.391 :theta 330}
                        {:z 353.682 :theta 325}
                        {:z 366.683 :theta 320}
                        {:z 379.229 :theta 315}
                        {:z 391.159 :theta 310}
                        {:z 412.583 :theta 300}
                        {:z 421.816 :theta 295}
                        {:z 429.914 :theta 290}
                        {:z 436.786 :theta 285}
                        {:z 442.360 :theta 280}
                        {:z 446.581 :theta 275}]]
    (doseq [{:keys [z theta]} physical-model]
      (println "z:" z "expected;" ((comp - unwrap) theta)
               "computed:" (Math/round (:theta1 (delta-calc-inverse 0 0 z)))))))
