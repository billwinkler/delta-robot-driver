(ns delta-robot.command-driver-test
  "Sequencing tests for streamed waveform transmission, run against a fake
  pigs so no pigpiod is needed. Verifies the two audited bugs stay fixed:
  1. CB exhaustion — never more than 2 waves alive, however long the move.
  2. Deletion race — send-commands is synchronous; every wave it creates
     is deleted before it returns, so no stale future can delete a
     successor move's re-used wave ids."
  (:require [clojure.test :refer [deftest is testing]]
            [delta-robot.command-driver :as cd]))

(defn- make-fake-pigs
  "Simulates the pigpiod wave lifecycle. `limit-after` (optional): the fake
  returns \"0\" (triggered) for limit-switch reads after that many reads.
  Returns {:fn ... :calls ... :state ...}."
  [& {:keys [limit-after] :or {limit-after Long/MAX_VALUE}}]
  (let [calls (atom [])
        limit-reads (atom 0)
        state (atom {:next-id 0 :playing nil :queued nil
                     :alive #{} :max-alive 0 :busy-drain 1})]
    {:calls calls
     :state state
     :fn (fn [& args]
           (swap! calls conj (vec args))
           (let [[cmd a b] args]
             (case cmd
               "wvag" "0"
               "wvcre" (let [id (:next-id @state)]
                         (swap! state
                                (fn [s]
                                  (let [s (-> s
                                              (update :next-id inc)
                                              (update :alive conj id))]
                                    (assoc s :max-alive
                                           (max (:max-alive s) (count (:alive s)))))))
                         (str id))
               "wvtxm" (let [wid (Long/parseLong a)]
                         (if (= "0" b)
                           (swap! state assoc :playing wid)
                           (swap! state assoc :queued wid))
                         "0")
               "wvcha" (do (swap! state assoc :playing (Long/parseLong (last args)))
                           "0")
               ;; each wvtat poll advances the simulation: the playing wave
               ;; finishes and any queued wave starts
               "wvtat" (let [{:keys [playing queued]} @state]
                         (if queued
                           (do (swap! state assoc :playing queued :queued nil)
                               (str queued))
                           (if playing (str playing) "9999")))
               ;; report busy once, then finished
               "wvbsy" (if (pos? (:busy-drain @state))
                         (do (swap! state update :busy-drain dec) "1")
                         (do (swap! state assoc :playing nil) "0"))
               "wvdel" (do (swap! state update :alive disj (Long/parseLong a)) "0")
               "wvhlt" (do (swap! state assoc :playing nil :queued nil) "0")
               "wvclr" (do (swap! state assoc :alive #{}) "0")
               "w" "0"
               "r" (do (swap! limit-reads inc)
                       (if (> @limit-reads limit-after) "0" "1"))
               "0")))}))

(defn- cmds-of [calls cmd] (filter #(= cmd (first %)) @calls))

(deftest giant-move-splits-into-sequential-chains
  (testing "a move over the chain budget splits at the midpoint: two
    seamless chains, never streaming, bounded live waves"
    (let [{fake :fn :keys [calls state]} (make-fake-pigs)]
      (with-redefs [cd/execute-pigs-cmd fake]
        (let [result (cd/send-commands {0 {:total-pulses 5000 :direction 0}
                                        1 {:total-pulses 5001 :direction 0}
                                        2 {:total-pulses 5002 :direction 0}})]
          (is (= :ok result))
          (is (empty? (cmds-of calls "wvtxm")) "streaming is gone")
          (is (= 2 (count (cmds-of calls "wvcha"))) "two sequential chains")
          (is (<= (:max-alive @state) 45)
              "CB budget: only one half-move's waves alive at once")
          (is (empty? (:alive @state)) "all waves deleted"))))))

(deftest short-move-single-chunk
  (testing "a move that fits one chunk: create, transmit, wait, delete"
    (let [{:keys [fn calls state]} (make-fake-pigs)]
      (with-redefs [cd/execute-pigs-cmd fn]
        (is (= :ok (cd/send-commands {0 {:total-pulses 20 :direction 0}
                                      1 {:total-pulses 20 :direction 0}
                                      2 {:total-pulses 20 :direction 0}})))
        (is (= 1 (count (cmds-of calls "wvcre"))))
        (is (empty? (:alive @state)))))))

(deftest limit-switch-aborts-split-move
  (testing "an upward giant move whose limit trips during the first half
    halts, cleans up, and never starts the second half"
    ;; guard reads pass twice (outer + first-half recursion = 6 reads),
    ;; then the trip happens during the first half's final-wait
    (let [{fake :fn :keys [calls state]} (make-fake-pigs :limit-after 6)]
      (with-redefs [cd/execute-pigs-cmd fake]
        (let [result (cd/send-commands {0 {:total-pulses 5000 :direction 1}
                                        1 {:total-pulses 5001 :direction 1}
                                        2 {:total-pulses 5002 :direction 1}})]
          (is (= :aborted result))
          (is (seq (cmds-of calls "wvhlt")) "waveform halted")
          (is (= 1 (count (cmds-of calls "wvcha"))) "second half never starts")
          (is (empty? (:alive @state)) "waves cleaned up after abort"))))))

(deftest small-move-plays-one-seamless-chain
  (testing "moves under the chain threshold pre-build all waves and play
    them as a single wvcha — one motor start per move, no inter-chunk
    stalls (the 70mm audit-drift bug: per-chunk cold starts lose steps)"
    (let [{fake :fn :keys [calls state]} (make-fake-pigs)]
      (with-redefs [cd/execute-pigs-cmd fake]
        (let [result (cd/send-commands {0 {:total-pulses 300 :direction 0}
                                        1 {:total-pulses 301 :direction 0}
                                        2 {:total-pulses 302 :direction 0}})]
          (is (= :ok result))
          (is (empty? (cmds-of calls "wvtxm")) "no streaming on small moves")
          (is (= 1 (count (cmds-of calls "wvcha"))) "exactly one chain")
          (let [order (map first @calls)
                cre-idxs (keep-indexed (fn [i c] (when (= c "wvcre") i)) order)
                cha-idx (first (keep-indexed (fn [i c] (when (= c "wvcha") i)) order))]
            (is (every? #(< % cha-idx) cre-idxs)
                "every wave is created before the chain starts"))
          (is (empty? (:alive @state)) "all waves deleted after the move"))))))

(deftest small-move-chain-limit-abort
  (testing "a limit trip during a chained move halts and cleans up"
    (let [{fake :fn :keys [calls state]} (make-fake-pigs :limit-after 3)]
      (with-redefs [cd/execute-pigs-cmd fake]
        (let [result (cd/send-commands {0 {:total-pulses 300 :direction 1}
                                        1 {:total-pulses 301 :direction 1}
                                        2 {:total-pulses 302 :direction 1}})]
          (is (= :aborted result))
          (is (seq (cmds-of calls "wvhlt")))
          (is (empty? (:alive @state))))))))

(deftest no-movement-is-a-noop
  (let [{:keys [fn calls]} (make-fake-pigs)]
    (with-redefs [cd/execute-pigs-cmd fn]
      (is (= :ok (cd/send-commands {0 {:total-pulses 0 :direction 0}
                                    1 {:total-pulses 0 :direction 0}
                                    2 {:total-pulses 0 :direction 0}})))
      (is (empty? (cmds-of calls "wvcre"))))))
