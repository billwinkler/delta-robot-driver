(ns delta-robot.kinematics-test
  (:require [clojure.test :refer :all]
            [delta-robot.inverse-kinematics :refer :all]
            [delta-robot.config :as cfg]))

(defn approximately-equal
  "Checks if two theta maps are equal within a tolerance."
  [expected actual tolerance]
  (and (< (Math/abs (- (:theta1 expected) (:theta1 actual))) tolerance)
       (< (Math/abs (- (:theta2 expected) (:theta2 actual))) tolerance)
       (< (Math/abs (- (:theta3 expected) (:theta3 actual))) tolerance)))

(deftest test-inverse-kinematics
  (let [tolerance 1.0
        in->mm #(* % 25.4)]
    ;; Physical measurements from the robot
    (testing "Physical robot positions"
      (is (approximately-equal {:theta1 28 :theta2 28 :theta3 28}
                              (delta-calc-inverse 0 0 (in->mm 8))
                              tolerance)
          "Retracted position should yield theta = 28°")
      (is (approximately-equal {:theta1 0 :theta2 0 :theta3 0}
                              (delta-calc-inverse 0 0 (in->mm 10.625))
                              tolerance)
          "Horizontal position should yield theta = 0°")
      (is (approximately-equal {:theta1 -90 :theta2 -90 :theta3 -90}
                              (delta-calc-inverse 0 0 (in->mm 17.32))
                              tolerance)
          "Extended position should yield theta = -90°"))
    ;; Selected measurements from Fusion 360 model
    (testing "Model positions"
      (is (approximately-equal {:theta1 25 :theta2 25 :theta3 25}
                              (delta-calc-inverse 0 0 217.098)
                              tolerance)
          "z=217.098 mm should yield theta = 25°")
      (is (approximately-equal {:theta1 0 :theta2 0 :theta3 0}
                              (delta-calc-inverse 0 0 263.500)
                              tolerance)
          "z=263.500 mm should yield theta = 0°")
      (is (approximately-equal {:theta1 -5 :theta2 -5 :theta3 -5}
                              (delta-calc-inverse 0 0 275.362)
                              tolerance)
          "z=275.362 mm should yield theta = -5°")
      (is (approximately-equal {:theta1 -85 :theta2 -85 :theta3 -85}
                              (delta-calc-inverse 0 0 446.581)
                              tolerance)
          "z=446.581 mm should yield theta = -85°")))
    ;; Test for an invalid position (outside workspace)
    (testing "Invalid position"
      (is (nil? (delta-calc-inverse 0 0 1000))
          "z=1000 mm should be unreachable and return nil"))))
