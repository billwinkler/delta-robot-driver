(ns delta-robot.core
  (:require [delta-robot.config :as cfg]
            [delta-robot.inverse-kinematics :as ik]
            [clojure.tools.logging :as log]
            [delta-robot.state :as state]))

;; We maintain current motor angles in an atom (in degrees)
;; Assume initial state is fully retracted
(def current-angles
  (atom
   (if-let [loaded-state (state/load-state)]
     (:current-angles loaded-state)
     (vec (repeat 3 (:max-angle cfg/config))))))

;; Convert an angular difference (in degrees) to motor pulses.
(defn deg->pulses [deg]
  (let [{:keys [steps-per-rev gear-ratio]} cfg/config]
    (Math/round (* deg (/ steps-per-rev 360.0) gear-ratio))))

(defn clamp [x]
  (let [{:keys [min-angle max-angle]} cfg/config]
    (max min-angle (min x max-angle))))

(defn compute-step-commands
  "For a given target (x, y, z), compute a sequence of motor commands.
   Returns a map with keys:
     :commands  - a vector of command maps for each motor (each with :motor-number, :total-pulses, and :direction).
     :new-angles - the target angles (after clamping) for each motor.
   Uses the current-angles atom, clamp, and deg->pulses helper functions."
  [x y z]
  (log/debug "Computing step commands for target:" [x y z])
  (log/debug "Starting current-angles:" @current-angles)
  (let [angles (ik/delta-calc-inverse x y z)]
    (if-not angles
      (throw (Exception. "Target position unreachable"))
      (let [{:keys [theta1 theta2 theta3]} angles
            desired-angles [theta1 theta2 theta3]
            clamped-angles (mapv clamp  desired-angles)
            _  (log/debug "desired-angles:" desired-angles)
            _  (log/debug "clamped-angles:" clamped-angles)
            angle-deltas (mapv - clamped-angles @current-angles)
            _  (log/debug "angle-deltas:" angle-deltas)
            commands (map-indexed
                      (fn [i delta]
                        (let [pulses (Math/abs (deg->pulses delta))
                              direction (if (pos? delta) 1 0)]
                          (log/debug "Motor" i "move:" delta "deg =>" pulses "pulses, direction:" direction)
                          {:motor-number i
                           :total-pulses pulses
                           :direction direction}))
                      angle-deltas)]
        (log/debug "Computed new angles:" clamped-angles)
        (log/debug "Commands:" commands)
        {:commands commands :new-angles clamped-angles}))))

(comment
  (ik/delta-calc-inverse 0 0 200)
  (ik/delta-calc-inverse 0 0 400)
  (ik/delta-calc-inverse 0 100 400)

  (deg->pulses 360)
  
  (compute-step-commands 0 0 200)
  (compute-step-commands 0 100 400)
  @current-angles
  )

