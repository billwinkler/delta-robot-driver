(ns delta-robot.grip-search-test
  (:require [clojure.test :refer [deftest is testing]]
            [delta-robot.grip-search :as gs]))

(deftest servo-target-test
  (testing "target is pecan + comp + nudge"
    (is (= [110.0 120.0] (gs/servo-target [110.0 120.0] [0.0 0.0] [0 0])))
    (is (= [99.0 105.0] (gs/servo-target [110.0 120.0] [-11.0 -15.0] [0 0])))
    (is (= [104.0 110.0] (gs/servo-target [110.0 120.0] [-11.0 -15.0] [5 5]))))
  (testing "nil when the pecan was not found"
    (is (nil? (gs/servo-target nil [-11.0 -15.0] [0 0])))))

(deftest cross-error-test
  (testing "error is target - cross"
    (is (= [10.0 20.0] (gs/cross-error [100.0 100.0] [110.0 120.0])))
    (is (= [0.0 0.0] (gs/cross-error [110.0 120.0] [110.0 120.0]))))
  (testing "nil when the cross was not found"
    (is (nil? (gs/cross-error nil [110.0 120.0])))))

(deftest error->move-test
  (let [cfg {:jinv [[0.412 0.147] [-0.203 0.642]]}
        servo {:gain 1.0 :max-step-mm 25}]
    (testing "identity-gain conversion matches Jinv"
      (let [[mx my] (gs/error->move [10.0 0.0] cfg servo)]
        (is (< (Math/abs (- mx 4.12)) 1e-9))
        (is (< (Math/abs (- my -2.03)) 1e-9))))
    (testing "gain scales the move"
      (let [[mx _] (gs/error->move [10.0 0.0] cfg (assoc servo :gain 0.5))]
        (is (< (Math/abs (- mx 2.06)) 1e-9))))
    (testing "moves are clamped to max-step-mm"
      (let [[mx my] (gs/error->move [1000.0 1000.0] cfg servo)]
        (is (== 25 mx))
        (is (<= (Math/abs (double my)) 25.0))))))

(deftest converged?-test
  (is (gs/converged? [3 4] 8))     ; magnitude 5
  (is (not (gs/converged? [6 8] 8)))  ; magnitude 10
  (is (gs/converged? [0 0] 1)))

(deftest next-position-test
  (testing "adds and rounds"
    (is (= [-45 10] (gs/next-position [-50 5] [5.4 4.6] 80))))
  (testing "clamps to workspace bound"
    (is (= [-80 80] (gs/next-position [-75 75] [-20 20] 80)))))

(deftest random-deposit-test
  (let [region {:x [-75 -35] :y [-25 25]}]
    (testing "extremes stay inside the region"
      (is (= [-75 -25] (gs/random-deposit region (constantly 0.0))))
      (is (= [-35 25] (gs/random-deposit region (constantly 0.999999)))))
    (testing "midpoint"
      (is (= [-55 0] (gs/random-deposit region (constantly 0.5)))))))

(deftest verdict-test
  (is (= :dropped (gs/verdict [700.0 450.0])))
  (is (= :lifted (gs/verdict nil))))

(deftest pecan-near-gap-test
  (testing "pecan within radius of the gap passes through"
    (is (= [860.0 540.0] (gs/pecan-near-gap [860.0 540.0] [858.0 506.5] 130))))
  (testing "a distant blob is a ghost, not the pecan"
    ;; the [552 456] static blob vs jaws at [858 506] — 313 px away
    (is (nil? (gs/pecan-near-gap [552.8 456.7] [858.0 506.5] 130))))
  (testing "exactly at radius still counts"
    (is (= [100.0 100.0] (gs/pecan-near-gap [100.0 100.0] [100.0 230.0] 130))))
  (testing "nil pecan or nil gap -> nil"
    (is (nil? (gs/pecan-near-gap nil [858.0 506.5] 130)))
    (is (nil? (gs/pecan-near-gap [860.0 540.0] nil 130)))))

(deftest graspable?-test
  (let [tips-h [[400.0 500.0] [460.0 500.0]]]   ; closing axis at 0 deg
    (testing "major axis perpendicular to the closing axis = graspable"
      (is (gs/graspable? tips-h 90.0 30))
      (is (gs/graspable? tips-h 70.0 30))     ; 70 deg off-axis, within tol
      (is (gs/graspable? tips-h 110.0 30)))
    (testing "long axis along the closing direction = jaws on the ends"
      (is (not (gs/graspable? tips-h 0.0 30)))
      (is (not (gs/graspable? tips-h 20.0 30)))
      (is (not (gs/graspable? tips-h 170.0 30))))  ; 170 ~ 10 deg undirected
    (testing "boundary: exactly at tolerance passes"
      (is (gs/graspable? tips-h 60.0 30))
      (is (not (gs/graspable? tips-h 59.0 30)))))
  (testing "diagonal closing axis"
    (let [tips-d [[400.0 400.0] [450.0 450.0]]]  ; 45 deg
      (is (gs/graspable? tips-d 135.0 30))
      (is (not (gs/graspable? tips-d 45.0 30)))))
  (testing "unmeasurable inputs -> nil (caller treats as don't-block)"
    (is (nil? (gs/graspable? [nil [460.0 500.0]] 90.0 30)))
    (is (nil? (gs/graspable? [[400.0 500.0] [460.0 500.0]] nil 30)))))

(deftest final-verdict-test
  (testing "a lifted verdict with a failed place check is downgraded"
    (is (= :lift-unverified (gs/final-verdict :lifted false))))
  (testing "a confirmed place keeps :lifted"
    (is (= :lifted (gs/final-verdict :lifted true))))
  (testing "an unmeasurable aim (nil) keeps :lifted"
    (is (= :lifted (gs/final-verdict :lifted nil))))
  (testing "non-lift verdicts pass through untouched"
    (is (= :dropped (gs/final-verdict :dropped nil)))
    (is (= :missed (gs/final-verdict :missed false)))))

(deftest gap-error-test
  (testing "error is pecan - gap (drive the gap onto the pecan)"
    (is (= [2.0 33.5] (gs/gap-error [858.0 506.5] [860.0 540.0])))
    (is (= [0.0 0.0] (gs/gap-error [858.0 506.5] [858.0 506.5]))))
  (testing "nil when either is missing"
    (is (nil? (gs/gap-error nil [860.0 540.0])))
    (is (nil? (gs/gap-error [858.0 506.5] nil)))))

(deftest validate-params-test
  (let [ok {:mm 25 :z-grip 417 :pause-ms 300 :dx-px 0 :dy-px 0}]
    (is (nil? (gs/validate-params ok)))
    (is (some? (gs/validate-params (assoc ok :mm 11))))
    (is (some? (gs/validate-params (assoc ok :mm 41))))
    (is (some? (gs/validate-params (assoc ok :z-grip 421))))
    (is (some? (gs/validate-params (assoc ok :z-grip 399))))
    (is (some? (gs/validate-params (assoc ok :pause-ms 5000))))
    (is (some? (gs/validate-params (assoc ok :dx-px -21))))
    (is (some? (gs/validate-params (assoc ok :dy-px 21))))))
