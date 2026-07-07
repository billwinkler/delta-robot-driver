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
   ;; after any camera move. Fit 2026-07-05 on the rigid mount by
   ;; LEAST SQUARES over attempt n9's verify-loop (move -> jaw-gap
   ;; displacement) pairs — measured at grip depth in the platform
   ;; plane, i.e. exactly the operating regime. The earlier 2-point
   ;; cross probe fit had a poisoned y-column (occluded cross fix):
   ;; its 0.772 cross-term coupled y-error into runaway x-moves
   ;; (n8 oscillation, n9 verify circling).
   ;; RE-FIT 2026-07-05 evening post arm-reattachment (kinematics
   ;; changed: old J predicted +37 px y-coupling on a 79 mm x-move,
   ;; reality is ~+8 px — photo-verified tool position error ~20 mm).
   ;; 3-point open-jaw gap probe at depth: +30x -> (+56,+3) px,
   ;; +25y -> (-8,+26.5) px.
   ;; RE-FIT 2026-07-07 on the NEW elevated-oblique scene mount:
   ;; 3-point open-jaw gap probe at depth z=417 around (30,10):
   ;; +25x -> (+38.5,+14) px, -25y -> (+17,-25.5) px, i.e.
   ;; J = [[1.54 -0.68] [0.56 1.02]] px/mm (~1.2-1.6 px/mm scale);
   ;; gap repeatability 0.5 px; hover probe agreed (x-col 1.53,0.60).
   :jinv [[0.523 0.348] [-0.287 0.789]]
   ;; SERVO SIGNAL CHANGE (2026-07-05 night): the servo now tracks the
   ;; JAW GAP-CENTER, not the laser cross. Post arm-reattachment the
   ;; cross detection became unstable (rigid cross-gap offset swung
   ;; +-60 px between frames; the tilted beams also shift ~42 px
   ;; between hover and depth) while the gap tracked flawlessly. The
   ;; cross was only ever a proxy for the gripper from before jaw
   ;; tips were detectable; the gap IS the gripper. comp-px is now
   ;; the HOVER->DEPTH tip parallax (gap_hover - gap_depth, measured
   ;; at staging): servo the hover gap onto pecan+comp so the tips
   ;; land ON the pecan when they descend. Re-measure after effector
   ;; work: hover + depth capture at staging.
   ;; RE-MEASURED 2026-07-07 (new mount): hover (733,412.5) ->
   ;; depth (728,430.5) at staging (30,10), z 400->417.
   :comp-px [5.0 -18.0]
   ;; PECAN HEIGHT PARALLAX (Bill's catch, 2026-07-05 night, from a
   ;; zoomed photo: "the gripper is always offset from center"): the
   ;; pecan's image CENTROID rides ~8.5 mm up the nut and projects
   ;; camera-away from its true FOOTPRINT center on the platform —
   ;; while the jaw tips are measured at platform level. Every
   ;; "converged" close was ~5-14 mm off, always the same direction.
   ;; Derived from the measured tip parallax ((-4,-21.5) px per 14 mm
   ;; of height): footprint = centroid + 8.5/14 * (4, 21.5).
   ;; RE-DERIVED 2026-07-07 (new mount): tip parallax (+5,-18) px
   ;; per 17 mm of height -> footprint = centroid + 8.5/17*(-5, 18).
   :pecan-height-comp-px [-2.5 9.0]
   :hover-z 400
   :lift-z 385
   ;; servo tolerance is SCALE-BOUND: 8 px was 1.3 mm at the old
   ;; 6 px/mm camera; at the rigid mount's 1.8 px/mm it demanded
   ;; ~4 mm from a coarse loop and oscillated just outside it
   ;; (2026-07-05 n8: iters 5-8 all within 9-15 px). The cross servo
   ;; only needs to deliver the arm near the pecan — the verify loop
   ;; at grip depth is the precision stage. Gain lowered to damp
   ;; oscillation from residual Jacobian error.
   :servo {:gain 0.6 :tol-px 18 :max-iters 10 :max-step-mm 25}
   ;; descend-verify-correct (2026-07-04): at grip depth the jaw tips
   ;; and the pecan share the platform plane, so gap-center vs pecan
   ;; is a parallax-free error — measured, not calibrated. comp-px is
   ;; demoted to an initial guess for the cross servo.
   ;; :pecan-radius-px doubles as ghost rejection: a "pecan" further
   ;; than this from the jaws is a static dark blob, not the target.
   ;; verify corrections run at FULL gain: the Jacobian is LS-fit
   ;; from at-depth data, and every extra raise-descend cycle is
   ;; another chance for an open jaw tip to clip and shove the pecan
   ;; (n11 chased its own nudges to the workspace clamp). tol 15 px
   ;; ~ 8 mm — 40 mm open jaws on a ~20 mm pecan still capture that.
   ;; :orient-tol-deg: pecan major axis must be perpendicular to the
   ;; jaw closing axis within this, else :bad-orientation (no descend,
   ;; no bump — a human must reposition; the arm has no wrist)
   ;; tol tightened 15->8 px (~4.5 mm) after n18: hard pads eject a
   ;; convex nut at ~3.5 mm off-center — shrink the squirt window
   ;; :pecan-radius-px rescaled 130->60 (2026-07-07): the constant
   ;; was calibrated at the old 6 px/mm camera (130 px = 22 mm); at
   ;; this mount's ~1.4 px/mm it meant 93 mm and admitted the ARM'S
   ;; DAYLIGHT SHADOW as a "pecan" 120 px from the gap (reliability
   ;; trial 11: verify oscillated between the shadow and :pre for 5
   ;; corrections). 60 px ~ 43 mm still accepts real strike-rolls
   ;; (26-52 px observed); a >60 px roll degrades to :pre + honest
   ;; :missed, recovered by the next attempt's fresh precheck.
   :verify {:tol-px 8 :max-corrections 5 :pecan-radius-px 60
            :gain 1.0
            ;; 45 = advisory-grade: the perpendicularity test compares
            ;; IMAGE angles, and the oblique camera distorts them (a
            ;; pecan Bill judged perfectly graspable read 55 deg).
            ;; Proper fix is a platform homography; until then only
            ;; block egregious long-axis-along-jaws cases.
            :orient-tol-deg 45}
   ;; workspace clamp for ALL commanded xy (CLI enforces its own too)
   :xy-bound 80
   ;; servo/nudge staging spot (arm xy). POSE-BOUND: must put the
   ;; hover jaw gap comfortably INSIDE the platform quad or tip
   ;; detection refuses (:no-gap — n9/n10 both died at the old
   ;; [-55 0], whose jaws fall outside the 2026-07-07 mount's quad).
   ;; (30,10) is the 2026-07-07 probe spot: gap verified detectable
   ;; at hover AND depth there.
   :staging [30 10]
   ;; radius of the jaw-tip detector's pecan-exclusion disc (fires on
   ;; tip-detection failure OR a tip landing inside the disc, i.e.
   ;; pecan-as-tip contamination). 26 = pecan blob half-extent +
   ;; margin at the 2026-07-07 scale, and safely under the ~29 px
   ;; distance of a GENUINE tip from the pecan centroid at converged
   ;; hover — the disc can never eat a real tip.
   :pecan-exclude-r-px 26
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

(defn footprint
  "Project a pecan image CENTROID down to its platform FOOTPRINT
  center (see :pecan-height-comp-px). Every aiming target must use
  the footprint; the raw centroid is only right for detection radii."
  [pecan]
  (when pecan
    (mapv + pecan (:pecan-height-comp-px cfg))))

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

(defn graspable?
  "Is the pecan oriented so the jaws can capture its waist?

  The wristless delta arm closes its jaws along a fixed axis — the
  line between the two jaw tips. The pecan is an ellipsoid: only its
  ~20 mm waist fits the jaws; if its LONG axis lies along the closing
  direction the jaws meet the tapered ends and slip is guaranteed
  (Bill's observation, 2026-07-05 — and any effector bump can roll it
  into that state). Graspable = pecan major axis perpendicular to the
  tip-to-tip line within tol degrees. Angles in image degrees, both
  measured from the same frame, so camera pose cancels out."
  [[tip1 tip2] pecan-angle tol-deg]
  (when (and tip1 tip2 pecan-angle)
    (let [jaw-deg (Math/toDegrees
                   (Math/atan2 (- (second tip2) (second tip1))
                               (- (first tip2) (first tip1))))
          ;; difference of undirected axes, folded into [0, 90]
          d (Math/abs (rem (- pecan-angle jaw-deg) 180.0))
          d (min d (- 180.0 d))]
      (>= d (- 90.0 tol-deg)))))

(defn rotation-needed
  "Signed degrees to rotate the pecan's major axis so it becomes
  perpendicular to the jaw closing axis (= graspable). Smallest
  magnitude, in [-90, 90). Zero-ish means already graspable."
  [pecan-angle jaw-axis-deg]
  (let [target (mod (+ jaw-axis-deg 90.0) 180.0)
        d (mod (- target pecan-angle) 180.0)]
    (if (>= d 90.0) (- d 180.0) d)))

(defn px->mm
  "Raw image-px displacement -> arm-mm move via Jinv (no gain, no
  clamp — plain geometry, unlike error->move)."
  [[ex ey] [[a b] [c d]]]
  [(+ (* a ex) (* b ey)) (+ (* c ex) (* d ey))])

(defn nudge-plan
  "Plan one rotation nudge in image space: sweep the closed-jaw tool
  along the perpendicular of the pecan's major axis THROUGH one of
  its ends — force at the end = torque about the center = yaw.

  Torque sign: r x F with r = s*offset*u and F along R90(u) gives
  tau = s*offset, so the push DIRECTION is always +R90(u) and the
  sign of the needed rotation picks WHICH END (s): pushing the +u
  end rotates +, the -u end rotates -. Both angles live in the same
  image frame, so camera pose cancels.

  Returns {:from [px] :to [px]} — sweep start (clear of the pecan)
  and end (through and past the end point)."
  [pecan-px pecan-angle dtheta {:keys [end-offset-px approach-px follow-px]}]
  (let [th (Math/toRadians pecan-angle)
        u [(Math/cos th) (Math/sin th)]
        p [(- (second u)) (first u)]          ; +R90(u), always
        s (if (pos? dtheta) 1.0 -1.0)
        e [(+ (first pecan-px) (* s end-offset-px (first u)))
           (+ (second pecan-px) (* s end-offset-px (second u)))]]
    {:from [(- (first e) (* approach-px (first p)))
            (- (second e) (* approach-px (second p)))]
     :to   [(+ (first e) (* follow-px (first p)))
            (+ (second e) (* follow-px (second p)))]}))

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
  "Capture + analyze one scene frame. Returns {:cross [..] :pecan [..]}.
  exclude-px: optional [x y] whose disc the jaw-tip detector may
  remove as a FALLBACK when plain detection fails — pass the known
  (unmoved) pecan position so a pecan visually merged into the jaw
  blob can't collapse the two-tip profile (see find_jaw_tips)."
  ([tag] (vision! tag nil))
  ([tag exclude-px]
   (let [dir (:frame-dir cfg)
         _ (io/make-parents (str dir "/x"))
         path (str dir "/" (System/currentTimeMillis) "-" tag ".jpg")
         args (concat [(:python cfg) (:vision-script cfg) "--save" path]
                      (when exclude-px
                        ["--exclude" (str (first exclude-px) ","
                                          (second exclude-px) ","
                                          (:pecan-exclude-r-px cfg 32))]))
         out (:out (apply shell {:out :string} args))]
     (json/parse-string out true))))

(defn servo-cross-to!
  "Iteratively drive the laser cross to a fixed scene-px target.
  Only the cross is detected per iteration (jaw-immune).
  Returns {:status :converged/:no-cross/:max-iters
           :xy [x y] :iters n :final-err [ex ey]}."
  [start-xy target & [pecan-px]]
  (let [{:keys [servo hover-z xy-bound]} cfg]
    (loop [xy start-xy, i 0]
      (let [v (vision! (str "servo-" i) pecan-px)
            signal (:gap-center v)
            err (cross-error signal target)]
        (cond
          (nil? signal) {:status :no-gap :xy xy :iters i}
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
        {:keys [tol-px max-corrections pecan-radius-px
                orient-tol-deg]} verify]
    (loop [xy start-xy, k 0, checks []]
      (motion/move-to (first xy) (second xy) z-grip)
      (let [v (vision! (str "verify-" k))
            gap (:gap-center v)
            seen (pecan-near-gap (:pecan v) gap pecan-radius-px)
            pecan (or seen (pecan-near-gap pre-pecan gap pecan-radius-px))
            err (gap-error gap (footprint pecan))
            orient-ok (if (and seen (:jaws v) (:pecan-angle v))
                        (graspable? (:jaws v) (:pecan-angle v)
                                    orient-tol-deg)
                        true)   ; can't measure -> don't block
            check {:k k :xy xy :gap gap :pecan (:pecan v)
                   :near-pecan pecan
                   :pecan-src (cond seen :frame pecan :pre :else nil)
                   :pecan-angle (:pecan-angle v)
                   :graspable orient-ok
                   :err err}
            checks (conj checks check)]
        (cond
          ;; jaws not measurable at depth (merged with pecan/shadow):
          ;; likely touching or very close -> close and let the grip
          ;; frame + lift verdict tell the truth
          (nil? gap)
          {:status :assume-contact :xy xy :k k :checks checks}

          ;; the pecan's long axis lies along the closing direction:
          ;; the jaws would meet the tapered ends. No parameter fixes
          ;; this and further descents only roll it more — stop and
          ;; report so the operator repositions.
          (not orient-ok)
          {:status :bad-orientation :xy xy :k k :checks checks}

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
          (let [params (assoc servo :gain (:gain verify 1.0))
                mv (error->move err cfg params)
                nxt (next-position xy mv xy-bound)]
            ;; retreat to hover before moving sideways
            (motion/move-to (first xy) (second xy) hover-z)
            (motion/move-to (first nxt) (second nxt) hover-z)
            (recur nxt (inc k) checks)))))))

(defn nudge-rotate!
  "Rotate the pecan toward a graspable orientation by sweeping the
  CLOSED jaws through its ends (nudge-plan). Closed loop: nudge,
  re-measure the angle from home, repeat. The wristless arm cannot
  rotate its grip — but it can rotate the WORKPIECE.

  Returns {:status :graspable | :gave-up | :no-target | :no-jaws
           :jaw-axis-deg d :nudges [...]}."
  [max-nudges]
  (let [{:keys [hover-z xy-bound jinv verify]} cfg
        push-z 414              ; tips ~3 mm off the paper: contact the
                                ; pecan's lower flank, below its center
        plan-cfg {:end-offset-px 22 :approach-px 38 :follow-px 26}
        staging (:staging cfg)
        clamp-xy (fn [[x y]] [(clamp (Math/round (double x)) (- xy-bound) xy-bound)
                              (clamp (Math/round (double y)) (- xy-bound) xy-bound)])]
    ;; measure the closing axis with jaws OPEN — closed jaws merge
    ;; into one finger and the tip-pair detector rightly refuses.
    ;; The tip-line angle is the same open or closed (symmetric jaws).
    (gripper/open)
    (motion/home)
    (motion/move-to (first staging) (second staging) hover-z)
    (let [va (vision! "nudge-axis")
          tips (:jaws va)
          ;; anchor the px->arm map at PUSH DEPTH: the hover gap
          ;; carries ~17 mm of parallax and made nudges sweep air
          ;; beside the pecan (first live run: k0 hit by luck, k1-k4
          ;; identical misses, center frozen at 514.5 px)
          _ (motion/move-to (first staging) (second staging) push-z)
          vd (vision! "nudge-anchor")
          gap0 (or (:gap-center vd) (:gap-center va))]
      (motion/move-to (first staging) (second staging) hover-z)
      (motion/home)
      (gripper/close)      ; now become the pusher tool
      (if-not (and tips gap0)
        {:status :no-jaws :nudges []}
        (let [[[x1 y1] [x2 y2]] tips
              jaw-deg (Math/toDegrees (Math/atan2 (- y2 y1) (- x2 x1)))]
          (loop [k 0, nudges []]
            (let [m (vision! (str "nudge-measure-" k))
                  pecan (:pecan m)
                  angle (:pecan-angle m)
                  need (when (and pecan angle)
                         (rotation-needed angle jaw-deg))]
              (cond
                (nil? pecan)
                {:status :no-target :jaw-axis-deg jaw-deg :nudges nudges}

                (nil? angle)
                {:status :no-angle :jaw-axis-deg jaw-deg :nudges nudges}

                (<= (Math/abs (double need)) (:orient-tol-deg verify))
                {:status :graspable :jaw-axis-deg jaw-deg
                 :final-angle angle :nudges nudges}

                (>= k max-nudges)
                {:status :gave-up :jaw-axis-deg jaw-deg
                 :final-angle angle :nudges nudges}

                :else
                (let [plan (nudge-plan (footprint pecan) angle need plan-cfg)
                      ;; anchor px->arm at staging: arm(p) ~ staging +
                      ;; Jinv*(p - gap0) — coarse is fine, the loop
                      ;; re-measures after every sweep
                      to-arm (fn [p] (clamp-xy
                                      (mapv + staging
                                            (px->mm (mapv - p gap0) jinv))))
                      from-xy (to-arm (:from plan))
                      to-xy (to-arm (:to plan))]
                  (motion/move-to (first from-xy) (second from-xy) hover-z)
                  (motion/move-to (first from-xy) (second from-xy) push-z)
                  (motion/move-to (first to-xy) (second to-xy) push-z)
                  (motion/move-to (first to-xy) (second to-xy) hover-z)
                  (motion/home)
                  (recur (inc k)
                         (conj nudges {:k k :angle angle :need need
                                       :pecan pecan
                                       :from from-xy :to to-xy})))))))))))

(def nudge-spec
  {:max-nudges {:coerce :int :default 5
                :desc "Give up after this many sweeps (1..8)."}
   :notes {:desc "Free-text note recorded with the run."}})

(defn nudge
  "Rotate the pecan to a graspable orientation (see nudge-rotate!).
  Logs the run to data/nudges.edn."
  {:org.babashka/cli {:spec nudge-spec}}
  [{:keys [max-nudges notes]}]
  (let [t0 (System/currentTimeMillis)
        max-n (clamp (or max-nudges 5) 1 8)
        result (nudge-rotate! max-n)
        entry (assoc result :ts t0 :notes notes
                     :ms (- (System/currentTimeMillis) t0))]
    (gripper/open)
    (motion/home)
    (append-log! "data/nudges.edn" entry)
    (println (pr-str entry))
    entry))

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
                target (servo-target (footprint (:pecan pre)) (:comp-px cfg)
                                     [dx-px dy-px])]
            (if-not target
              (finish! {:verdict :no-target :pre pre})
              (do
                ;; move into the workspace before servoing
                (let [[sx sy] (:staging cfg)]
                  (motion/move-to sx sy (:hover-z cfg)))
                (let [servo (servo-cross-to! (:staging cfg) target
                                             (:pecan pre))]
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
                      (if (= :bad-orientation (:status vr))
                        ;; jaws would meet the tapered ends — no close,
                        ;; no lift; a human must reorient the pecan
                        (do (motion/home)
                            (finish! {:verdict :bad-orientation
                                      :pre (select-keys pre [:pecan :cross])
                                      :target target
                                      :servo (select-keys servo [:iters :final-err :xy])
                                      :verify vr-log}))
                        (do
                          (gripper/grip-hold mm)
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
                                        :place place}))))))))))))))))))
