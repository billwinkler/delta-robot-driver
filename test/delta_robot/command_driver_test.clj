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

(deftest long-move-bounded-live-waves
  (testing "a long gcd=1 move never holds more than 2 waves alive"
    (let [{:keys [fn state]} (make-fake-pigs)]
      (with-redefs [cd/execute-pigs-cmd fn]
        (let [result (cd/send-commands {0 {:total-pulses 500 :direction 0}
                                        1 {:total-pulses 501 :direction 0}
                                        2 {:total-pulses 502 :direction 0}})]
          (is (= :ok result))
          (is (<= (:max-alive @state) 2)
              "CB budget: at most playing + queued waves exist at once")
          (is (empty? (:alive @state))
              "all waves deleted before send-commands returns")
          (is (nil? (:queued @state))))))))

(deftest every-created-wave-transmitted-in-order
  (testing "chunks are created and queued in sequence: first oneshot, rest sync"
    (let [{:keys [fn calls]} (make-fake-pigs)]
      (with-redefs [cd/execute-pigs-cmd fn]
        (cd/send-commands {0 {:total-pulses 500 :direction 0}
                           1 {:total-pulses 501 :direction 0}
                           2 {:total-pulses 502 :direction 0}})
        (let [txs (cmds-of calls "wvtxm")
              modes (map #(nth % 2) txs)]
          (is (pos? (count txs)))
          (is (= "0" (first modes)) "first chunk starts immediately (oneshot)")
          (is (every? #(= "2" %) (rest modes)) "later chunks queue as oneshot-sync")
          (is (= (count (cmds-of calls "wvcre")) (count txs))
              "every created wave is transmitted"))))))

(deftest short-move-single-chunk
  (testing "a move that fits one chunk: create, transmit, wait, delete"
    (let [{:keys [fn calls state]} (make-fake-pigs)]
      (with-redefs [cd/execute-pigs-cmd fn]
        (is (= :ok (cd/send-commands {0 {:total-pulses 20 :direction 0}
                                      1 {:total-pulses 20 :direction 0}
                                      2 {:total-pulses 20 :direction 0}})))
        (is (= 1 (count (cmds-of calls "wvcre"))))
        (is (empty? (:alive @state)))))))

(deftest limit-switch-aborts-stream
  (testing "an upward move whose limit trips mid-stream halts and cleans up"
    ;; guard reads pass (3 reads, one per motor), then reads trip
    (let [{:keys [fn calls state]} (make-fake-pigs :limit-after 3)]
      (with-redefs [cd/execute-pigs-cmd fn]
        (let [result (cd/send-commands {0 {:total-pulses 2000 :direction 1}
                                        1 {:total-pulses 2001 :direction 1}
                                        2 {:total-pulses 2002 :direction 1}})]
          (is (= :aborted result))
          (is (seq (cmds-of calls "wvhlt")) "waveform halted")
          (is (empty? (:alive @state)) "waves cleaned up after abort"))))))

(deftest no-movement-is-a-noop
  (let [{:keys [fn calls]} (make-fake-pigs)]
    (with-redefs [cd/execute-pigs-cmd fn]
      (is (= :ok (cd/send-commands {0 {:total-pulses 0 :direction 0}
                                    1 {:total-pulses 0 :direction 0}
                                    2 {:total-pulses 0 :direction 0}})))
      (is (empty? (cmds-of calls "wvcre"))))))
