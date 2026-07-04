(ns delta-robot.grip-search
  "Grip-search harness: one fully-scripted grip attempt per invocation.

  Designed so a small model can drive the parameter search safely:
  every safety rail lives HERE, not in the driver. Each attempt starts
  from the fixed home position, servos the laser cross onto the pecan
  (scene camera; cross and pecan share the platform plane so there is
  no parallax), descends, grips, lifts, and issues an automated
  verdict. A successful lift deposits the pecan at a random reachable
  spot so the next attempt starts from a fresh configuration.

  Calibration constants (2026-07-04 session) live in `cfg`."
  (:require [babashka.cli :as cli]
            [babashka.process :refer [shell]]
            [cheshire.core :as json]
            [clojure.edn :as edn]
            [clojure.java.io :as io]
            [delta-robot.motion :as motion]
            [delta-robot.gripper :as gripper]))

(def cfg
  {:python "python/.venv/bin/python3"
   :vision-script "python/scene_vision.py"
   :log-file "data/grip-attempts.edn"
   :frame-dir "data/grip-frames"
   ;; scene-camera Jacobian inverse, mm per px (fit 2026-07-04)
   :jinv [[0.412 0.147] [-0.203 0.642]]
   ;; cross->grip-point compensation, scene px: servo target = pecan + comp.
   ;; Measured 2026-07-04 from a closed-jaws-at-grip-height frame:
   ;; jaw-gap center (942,500), cross (974,515) -> cross leads by (+32,+15)
   :comp-px [32.0 15.0]
   :hover-z 400
   :lift-z 385
   :servo {:gain 0.9 :tol-px 8 :max-iters 10 :max-step-mm 25}
   ;; workspace clamp for ALL commanded xy (CLI enforces its own too)
   :xy-bound 80
   ;; random deposit region (arm coords, comfortably on the platform)
   :deposit {:x [-75 -35] :y [-25 25]}
   :budget 20})

;; ---------------------------------------------------------------------
;; pure functions (unit tested)

(defn clamp [v lo hi] (max lo (min hi v)))

(defn servo-target
  "Fixed scene-px target for the cross: pecan + comp + nudge.
  The pecan is located ONCE from home (jaws out of frame — at hover
  they hang into the ROI and are pecan-sized dark blobs)."
  [pecan [cx cy] [nx ny]]
  (when pecan
    [(+ (first pecan) cx nx) (+ (second pecan) cy ny)]))

(defn cross-error
  "Pixel error the servo must null: target - cross."
  [cross [tx ty]]
  (when cross
    [(- tx (first cross)) (- ty (second cross))]))

(defn error->move
  "Convert a pixel error to a clamped arm move in mm via Jinv*err*gain."
  [[ex ey] {:keys [jinv]} {:keys [gain max-step-mm]}]
  (let [[[a b] [c d]] jinv
        mx (* gain (+ (* a ex) (* b ey)))
        my (* gain (+ (* c ex) (* d ey)))]
    [(clamp mx (- max-step-mm) max-step-mm)
     (clamp my (- max-step-mm) max-step-mm)]))

(defn converged? [[ex ey] tol-px]
  (<= (Math/sqrt (+ (* ex ex) (* ey ey))) tol-px))

(defn next-position
  "Next absolute xy, clamped to the workspace bound."
  [[x y] [dx dy] bound]
  [(clamp (Math/round (double (+ x dx))) (- bound) bound)
   (clamp (Math/round (double (+ y dy))) (- bound) bound)])

(defn random-deposit
  "Random xy inside the deposit region. rand-fn returns [0,1) doubles."
  [{:keys [x y]} rand-fn]
  [(Math/round (double (+ (first x) (* (rand-fn) (- (second x) (first x))))))
   (Math/round (double (+ (first y) (* (rand-fn) (- (second y) (first y))))))])

(defn verdict
  "Pecan visible on the platform after lift -> it was dropped."
  [pecan-detection]
  (if pecan-detection :dropped :lifted))

(defn validate-params [{:keys [mm z-grip pause-ms dx-px dy-px]}]
  (cond
    (not (<= 12 mm 40)) (str "mm must be 12..40, got " mm)
    (not (<= 400 z-grip 420)) (str "z-grip must be 400..420, got " z-grip)
    (not (<= 0 pause-ms 3000)) (str "pause-ms must be 0..3000, got " pause-ms)
    (not (<= -20 dx-px 20)) (str "dx-px must be -20..20, got " dx-px)
    (not (<= -20 dy-px 20)) (str "dy-px must be -20..20, got " dy-px)
    :else nil))

;; ---------------------------------------------------------------------
;; side-effecting plumbing

(defn read-log [file]
  (if (.exists (io/file file)) (edn/read-string (slurp file)) []))

(defn append-log! [file entry]
  (let [log (conj (read-log file) entry)]
    (io/make-parents file)
    (spit file (pr-str log))
    log))

(defn vision!
  "Capture + analyze one scene frame. Returns {:cross [..] :pecan [..]}."
  [tag]
  (let [dir (:frame-dir cfg)
        _ (io/make-parents (str dir "/x"))
        path (str dir "/" (System/currentTimeMillis) "-" tag ".jpg")
        out (:out (shell {:out :string}
                         (:python cfg) (:vision-script cfg) "--save" path))]
    (json/parse-string out true)))

(defn servo-cross-to!
  "Iteratively drive the laser cross to a fixed scene-px target.
  Only the cross is detected per iteration (jaw-immune).
  Returns {:status :converged/:no-cross/:max-iters
           :xy [x y] :iters n :final-err [ex ey]}."
  [start-xy target]
  (let [{:keys [servo hover-z xy-bound]} cfg]
    (loop [xy start-xy, i 0]
      (let [v (vision! (str "servo-" i))
            err (cross-error (:cross v) target)]
        (cond
          (nil? (:cross v)) {:status :no-cross :xy xy :iters i}
          (converged? err (:tol-px servo)) {:status :converged :xy xy
                                            :iters i :final-err err}
          (>= i (:max-iters servo)) {:status :max-iters :xy xy
                                     :iters i :final-err err}
          :else
          (let [mv (error->move err cfg servo)
                nxt (next-position xy mv xy-bound)]
            (motion/move-to (first nxt) (second nxt) hover-z)
            (recur nxt (inc i))))))))

;; ---------------------------------------------------------------------
;; the task

(def grip-attempt-spec
  {:mm {:coerce :int :require true
        :desc "Gripper closing opening in mm (12..40)."}
   :z-grip {:coerce :int :default 417
            :desc "Descend-to z before closing (400..420)."}
   :pause-ms {:coerce :int :default 300
              :desc "Settle pause between close and lift (0..3000)."}
   :dx-px {:coerce :int :default 0
           :desc "Servo target nudge in scene px, x (-20..20)."}
   :dy-px {:coerce :int :default 0
           :desc "Servo target nudge in scene px, y (-20..20)."}
   :notes {:desc "Free-text note recorded with the attempt."}})

(defn grip-attempt
  "Run one complete grip attempt. See ns docstring."
  {:org.babashka/cli {:spec grip-attempt-spec}}
  [{:keys [mm z-grip pause-ms dx-px dy-px notes] :as opts}]
  (let [t0 (System/currentTimeMillis)
        log-file (:log-file cfg)
        attempts (read-log log-file)
        n (inc (count attempts))
        base {:n n :ts t0
              :params (select-keys opts [:mm :z-grip :pause-ms :dx-px :dy-px])
              :notes notes}
        finish! (fn [entry]
                  (let [e (assoc entry :ms (- (System/currentTimeMillis) t0))]
                    (append-log! log-file (merge base e))
                    (println (pr-str (merge base e)))
                    e))]
    (if-let [err (validate-params opts)]
      (finish! {:verdict :invalid-params :error err})
      (if (> n (:budget cfg))
        (finish! {:verdict :budget-exhausted
                  :error (str "session budget " (:budget cfg) " reached")})
        (do
          ;; every trial starts from the fixed, known home position;
          ;; the pecan is located from HOME (jaws out of the scene ROI)
          (gripper/open)
          (motion/home)
          (let [pre (vision! "precheck")
                target (servo-target (:pecan pre) (:comp-px cfg)
                                     [dx-px dy-px])]
            (if-not target
              (finish! {:verdict :no-target :pre pre})
              (do
                ;; move into the workspace before servoing
                (motion/move-to -55 0 (:hover-z cfg))
                (let [servo (servo-cross-to! [-55 0] target)]
                  (if (not= :converged (:status servo))
                    (do (motion/home)
                        (finish! {:verdict :servo-failed :servo servo
                                  :pre (select-keys pre [:pecan :cross])
                                  :target target}))
                    (let [[x y] (:xy servo)]
                      (motion/move-to x y z-grip)
                      (gripper/grip mm)
                      (Thread/sleep 200)
                      (let [grip-v (vision! "grip")]
                        (Thread/sleep pause-ms)
                        (motion/move-to x y (:lift-z cfg))
                        (Thread/sleep 300)
                        ;; verdict is judged from HOME so the jaws are
                        ;; out of the scene ROI (pecan rides along if held)
                        (motion/home)
                        (let [lift-v (vision! "verdict")
                              v (verdict (:pecan lift-v))
                              deposit (when (= v :lifted)
                                        (random-deposit (:deposit cfg) rand))
                              placed (when deposit
                                       (let [[dx dy] deposit]
                                         (motion/move-to dx dy (:hover-z cfg))
                                         (motion/move-to dx dy 417)
                                         (gripper/open)
                                         (motion/move-to dx dy (:hover-z cfg))
                                         (motion/home)
                                         (vision! "deposit-check")))]
                          (when (= v :dropped) (gripper/open))
                          (motion/home)
                          (finish! {:verdict v
                                    :pre (select-keys pre [:pecan :cross])
                                    :target target
                                    :servo (select-keys servo [:iters :final-err :xy])
                                    :grip-vision grip-v
                                    :lift-vision lift-v
                                    :deposit deposit
                                    :deposit-check (when placed
                                                     (select-keys placed [:pecan]))}))))))))))))))
