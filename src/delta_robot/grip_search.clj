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
   ;; scene-camera Jacobian inverse, mm per px. POSE-BOUND: re-probe
   ;; after any camera move (2 probe moves, ~30 s). Fit 2026-07-05
   ;; after the re-aim (+20mm x -> (-96.6 +6.7) px; +20mm y ->
   ;; (-114.6 -150.3) px; camera now ~6 px/mm, was ~2.5).
   :jinv [[-0.1966 0.1499] [-0.0088 -0.1264]]
   ;; cross->grip-point compensation, scene px: servo target = pecan +
   ;; comp. Since descend-verify-correct (2026-07-04) this is only the
   ;; INITIAL guess — the verify loop measures the true jaw-gap-vs-
   ;; pecan offset at grip depth every attempt — so after the
   ;; 2026-07-05 camera re-aim it is simply zeroed rather than
   ;; re-measured; expect the verify loop to eat 1-2 extra corrections
   ;; on early attempts and refine from their logged checks if needed.
   :comp-px [0.0 0.0]
   :hover-z 400
   :lift-z 385
   :servo {:gain 0.9 :tol-px 8 :max-iters 10 :max-step-mm 25}
   ;; descend-verify-correct (2026-07-04): at grip depth the jaw tips
   ;; and the pecan share the platform plane, so gap-center vs pecan
   ;; is a parallax-free error — measured, not calibrated. comp-px is
   ;; demoted to an initial guess for the cross servo.
   ;; :pecan-radius-px doubles as ghost rejection: a "pecan" further
   ;; than this from the jaws is a static dark blob, not the target.
   :verify {:tol-px 12 :max-corrections 3 :pecan-radius-px 130}
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

(defn pecan-near-gap
  "The pecan detection, if it is within radius px of the jaw gap
  center — otherwise nil. Rejects static dark blobs (tape corners,
  platform holes) that the detector latches onto when the real pecan
  is occluded or shadowed: a 'pecan' far from the jaws is a ghost."
  [pecan gap radius]
  (when (and pecan gap)
    (let [dx (- (first pecan) (first gap))
          dy (- (second pecan) (second gap))]
      (when (<= (Math/sqrt (+ (* dx dx) (* dy dy))) radius)
        pecan))))

(defn gap-error
  "Pixel error to null in the verify step: pecan - gap-center.
  Same image plane as the cross servo, so the same Jinv applies."
  [gap pecan]
  (when (and gap pecan)
    [(- (first pecan) (first gap)) (- (second pecan) (second gap))]))

(defn final-verdict
  "Positive-evidence upgrade of the lift verdict (added after n7,
  2026-07-04: the from-home 'platform is empty' check reported :lifted
  while the operator was briefly holding the pecan — absence of the
  pecan is NOT evidence it is in the jaws). A :lifted verdict is
  confirmed only when the deposited pecan is later detected near the
  spot the deposit aimed at (:placed true). :placed false downgrades
  to :lift-unverified; :placed nil (aim cross not measurable) keeps
  :lifted but the log carries the gap in evidence."
  [lift-verdict placed?]
  (if (and (= :lifted lift-verdict) (false? placed?))
    :lift-unverified
    lift-verdict))

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

(defn descend-verify-correct!
  "Descend to grip depth with jaws OPEN and measure the actual
  jaw-gap-vs-pecan error in the platform plane; correct if needed.

  This replaces trust in the comp-px constant with a measurement:
  the cross servo (position-dependent comp) gets us close, then this
  loop measures where the jaws ACTUALLY are relative to the pecan and
  fixes the residual. Corrections happen at hover height so the open
  jaws never drag through the pecan sideways.

  When the frame's own pecan detection is missing or rejected (the
  pecan often merges into the jaw blob when they touch — live-verify
  n6 closed beside the pecan this way), the PRE-SERVO pecan position
  is used instead: the pecan hasn't moved since the precheck and the
  coordinates live in the same image plane, so gap vs pre-pecan is
  still a valid error. :assume-contact remains only for the case
  where not even the jaws are measurable.

  Returns {:status :verified | :assume-contact | :no-jaws | :max-corrections
           :xy [x y] :k n :checks [...]}  — :checks logs every
  measurement for the attempts log (Phase-4 training data)."
  [start-xy z-grip pre-pecan]
  (let [{:keys [verify hover-z xy-bound servo]} cfg
        {:keys [tol-px max-corrections pecan-radius-px]} verify]
    (loop [xy start-xy, k 0, checks []]
      (motion/move-to (first xy) (second xy) z-grip)
      (let [v (vision! (str "verify-" k))
            gap (:gap-center v)
            seen (pecan-near-gap (:pecan v) gap pecan-radius-px)
            pecan (or seen (pecan-near-gap pre-pecan gap pecan-radius-px))
            err (gap-error gap pecan)
            check {:k k :xy xy :gap gap :pecan (:pecan v)
                   :near-pecan pecan
                   :pecan-src (cond seen :frame pecan :pre :else nil)
                   :err err}
            checks (conj checks check)]
        (cond
          ;; jaws not measurable at depth (merged with pecan/shadow):
          ;; likely touching or very close -> close and let the grip
          ;; frame + lift verdict tell the truth
          (nil? gap)
          {:status :assume-contact :xy xy :k k :checks checks}

          ;; jaws measured but neither the frame's pecan nor the
          ;; pre-servo position is anywhere near them -> the target
          ;; is genuinely unaccounted for; bet on contact and let the
          ;; grip frame + lift verdict rule
          (nil? pecan)
          {:status :assume-contact :xy xy :k k :checks checks}

          (converged? err tol-px)
          {:status :verified :xy xy :k k :checks checks}

          (>= k max-corrections)
          {:status :max-corrections :xy xy :k k :checks checks}

          :else
          (let [mv (error->move err cfg servo)
                nxt (next-position xy mv xy-bound)]
            ;; retreat to hover before moving sideways
            (motion/move-to (first xy) (second xy) hover-z)
            (motion/move-to (first nxt) (second nxt) hover-z)
            (recur nxt (inc k) checks)))))))

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
                    ;; measure-and-correct at grip depth (jaws open):
                    ;; the jaw tips and the pecan share the platform
                    ;; plane, so this is a parallax-free measurement of
                    ;; where the jaws actually are — comp-px is only
                    ;; the initial guess that got us here.
                    (let [vr (descend-verify-correct! (:xy servo) z-grip
                                                      (:pecan pre))
                          [x y] (:xy vr)
                          vr-log (select-keys vr [:status :k :checks])]
                      (gripper/grip mm)
                      (Thread/sleep 200)
                      (let [grip-v (vision! "grip")
                            last-gap (or (:gap-center grip-v)
                                         (some :gap (reverse (:checks vr))))
                            missed (pecan-near-gap
                                    (:pecan grip-v) last-gap
                                    (get-in cfg [:verify :pecan-radius-px]))]
                        (if missed
                          ;; jaws demonstrably closed BESIDE the pecan:
                          ;; no point lifting — report the miss honestly
                          ;; so the driver learns targeting vs friction
                          (do (gripper/open)
                              (motion/home)
                              (finish! {:verdict :missed
                                        :pre (select-keys pre [:pecan :cross])
                                        :target target
                                        :servo (select-keys servo [:iters :final-err :xy])
                                        :verify vr-log
                                        :grip-vision grip-v}))
                          (do
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
                                  place (when deposit
                                          (let [[dx dy] deposit]
                                            (motion/move-to dx dy (:hover-z cfg))
                                            ;; the cross marks where the jaws
                                            ;; point: capture the aim BEFORE
                                            ;; releasing — the pecan must be
                                            ;; found near this spot afterward
                                            (let [aim (:cross (vision! "deposit-aim"))]
                                              (motion/move-to dx dy 417)
                                              (gripper/open)
                                              (motion/move-to dx dy (:hover-z cfg))
                                              (motion/home)
                                              (let [check (vision! "deposit-check")
                                                    found (:pecan check)]
                                                {:aim aim
                                                 :pecan found
                                                 :placed (when aim
                                                           (boolean
                                                            (pecan-near-gap
                                                             found aim
                                                             (get-in cfg [:verify :pecan-radius-px]))))}))))
                                  v-final (final-verdict v (:placed place))]
                              (when (= v :dropped) (gripper/open))
                              (motion/home)
                              (finish! {:verdict v-final
                                        :pre (select-keys pre [:pecan :cross])
                                        :target target
                                        :servo (select-keys servo [:iters :final-err :xy])
                                        :verify vr-log
                                        :grip-vision grip-v
                                        :lift-vision lift-v
                                        :deposit deposit
                                        :place place}))))))))))))))))
