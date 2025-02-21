(ns delta-robot.kinematics)

;; Geometry parameters (in mm)
(def pivot-x 34.64)    ;; Lower-arm pivot location x coordinate
(def pivot-z 0.0)      ;; Pivot is at z = 0 (Fusion’s origin is [0,0,0])
(def upper-arm 50.0)   ;; Upper arm length (from pivot to elbow)
(def lower-arm 80.0)   ;; Lower arm length (from elbow to effector joint)

;; Helper: given an upper-arm angle theta (in degrees, measured clockwise from +x),
;; compute the elbow position in the XZ plane.
(defn compute-elbow
  [theta-deg]
  (let [theta-rad (Math/toRadians (- theta-deg)) ; negate because Fusion angles increase clockwise
        ex (+ pivot-x (* upper-arm (Math/cos theta-rad)))
        ez (+ pivot-z (* upper-arm (- (Math/sin theta-rad))))]
    [ex ez]))

(defn compute-elbow [theta-deg]
  (let [theta-rad (Math/toRadians theta-deg)
        ;; If clockwise rotation should lower the elbow, subtract the sine term:
        ex (+ pivot-x (* upper-arm (Math/cos theta-rad)))
        ez (- pivot-z (* upper-arm (Math/sin theta-rad)))]
    [ex ez]))


;; Helper: given an elbow position [ex, ez] and a lower-arm orientation phi (in radians),
;; compute the effector joint position in XZ.
;; We assume that the lower arm “points to the left” from the elbow,
;; so we subtract 80*cos(phi) from ex and add 80*sin(phi) to ez.
(defn compute-effector-from-phi
  [elbow phi]
  (let [[ex ez] elbow
        fx (- ex (* lower-arm (Math/cos phi)))
        fz (+ ez (* lower-arm (Math/sin phi)))]
    [fx fz]))

;; Define an error function.
;; For a given phi, the error is the squared distance between
;; the computed effector F and the Fusion-measured target F_target.
(defn effector-error
  [phi elbow f-target]
  (let [[fx fz] (compute-effector-from-phi elbow phi)
        [tx tz] f-target]
    (+ (Math/pow (- fx tx) 2)
       (Math/pow (- fz tz) 2))))

;; A simple search function: given elbow and a target effector position,
;; find the phi (in radians) in a given range that minimizes the error.
(defn find-optimal-phi
  [elbow f-target]
  (let [; We'll search phi between 0 and PI (in 0.001 increments)
        phis (map #(/ % 1000.0) (range 0 3142))  ; from 0 to ~3.142 in steps of ~0.001
        candidate (reduce (fn [best phi]
                              (let [err (effector-error phi elbow f-target)]
                                (if (< err (:err best))
                                  {:phi phi :err err}
                                  best)))
                            {:phi nil :err Double/MAX_VALUE}
                            phis)]
    (:phi candidate)))

;; Now we create a function that, given theta (in degrees) and a Fusion-measured target effector F_target,
;; computes the elbow and then finds the lower-arm orientation phi that makes the computed effector as close as possible to F_target.
(defn compute-effector
  "For a given upper-arm angle theta-deg and a Fusion-measured target effector position f-target (a 2D vector [x z]),
   compute the elbow position and then numerically solve for the lower-arm orientation phi
   that minimizes the error, returning a map with theta, elbow, phi (in degrees), and computed effector."
  [theta-deg f-target]
  (let [elbow (compute-elbow theta-deg)
        phi-opt (find-optimal-phi elbow f-target)
        effector (compute-effector-from-phi elbow phi-opt)]
    {:theta theta-deg
     :elbow elbow
     :phi-deg (Math/toDegrees phi-opt)
     :effector effector}))

;; Example usage:
;; For theta = 25°, Fusion tells us that the effector joint should be at [8.66, 15.16] (in the XZ plane).
(def fusion-target-25 [8.66 15.16])
(def fusion-target-275 [8.66 123.834])


;; Compute the solution for theta = 25
(compute-effector 25 fusion-target-25)
;; => {:theta 25,
;;     :elbow [79.9553893518325 -21.130913087034973],
;;     :phi-deg 26.98631215066177,
;;     :effector [8.666192855726578 15.171297079008635]}

;; Compute the solution for theta = 275
(compute-effector 275 fusion-target-275)
;; => {:theta 275,
;;     :elbow [38.99778713738289 49.80973490458728],
;;     :phi-deg 67.7236113844633,
;;     :effector [8.671798999344592 123.83901589158296]}
