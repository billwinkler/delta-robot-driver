(ns delta-translator-test
  (:require [clojure.test :refer :all]
            [clojure.edn :as edn]
            [clojure.java.io :as io]
            [delta-translator :refer :all])
  (:import [java.nio ByteBuffer ByteOrder]))

;; Test that a single motor command is encoded correctly.
(deftest test-encode-motor-command
  (testing "Encode a single motor command"
    (let [motor {:total-pulses 1000
                 :target-freq 400
                 :accel-pulses 100
                 :decel-pulses 100
                 :direction 1}
          encoded (encode-motor-command motor)]
      (is (= [1000 400 100 100 1] encoded)))))

;; Test that a full command with multiple motors is encoded as expected.
(deftest test-encode-command
  (testing "Encode full command for multiple motors"
    (let [cmd {:motors [{:total-pulses 1000
                         :target-freq 400
                         :accel-pulses 100
                         :decel-pulses 100
                         :direction 1}
                        {:total-pulses 1200
                         :target-freq 450
                         :accel-pulses 110
                         :decel-pulses 110
                         :direction 0}]}
          buf (encode-command cmd)
          ;; Read out the ints from the ByteBuffer.
          ;; We expect: header: 2 motors, then the two motor parameter sequences.
          num-ints (/ (.limit buf) 4)
          ints (doall (for [_ (range num-ints)]
                        (.getInt buf)))]
      (is (= [2 1000 400 100 100 1 1200 450 110 110 0] ints)))))

;; Test the binary write function using a temporary file.
(deftest test-write-binary-message
  (testing "Write binary message to a temporary file"
    (let [cmd {:motors [{:total-pulses 1000
                         :target-freq 400
                         :accel-pulses 100
                         :decel-pulses 100
                         :direction 1}]}
          buf (encode-command cmd)
          ;; Create a temporary file.
          tmp-file (java.io.File/createTempFile "delta_test" ".bin")]
      (write-binary-message (.getAbsolutePath tmp-file) buf)
      ;; Now read back the file as bytes and check the content.
      (with-open [in (io/input-stream tmp-file)]
        (let [file-bytes (byte-array (.length tmp-file))]
          (.read in file-bytes)
          (let [buf-read (ByteBuffer/wrap file-bytes)]
            (.order buf-read ByteOrder/LITTLE_ENDIAN)
            (is (= 1 (.getInt buf-read)))       ; header: 1 motor
            (is (= 1000 (.getInt buf-read)))     ; motor: total-pulses
            (is (= 400 (.getInt buf-read)))      ; target-freq
            (is (= 100 (.getInt buf-read)))      ; accel-pulses
            (is (= 100 (.getInt buf-read)))      ; decel-pulses
            (is (= 1 (.getInt buf-read))))))
      (.delete tmp-file))))

;; Run the tests when using Babashka or nbb with the --test flag.
(run-tests)
