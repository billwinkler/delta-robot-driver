(ns delta-translator-test
  (:require [clojure.test :refer :all]
            [test-helper]
            [clojure.edn :as edn]
            [clojure.java.io :as io]
            [delta-translator :refer :all])
  (:import [java.nio ByteBuffer ByteOrder]))

;; Test that a single motor command is encoded correctly.
(deftest test-encode-motor-command
  (testing "Encode a single motor command"
    (let [motor {:motor-number 1
                 :total-pulses 1000
                 :direction 1}
          encoded (encode-motor-command motor)]
      (is (= [1 1000 1] encoded)))))

;; Test that a full command with multiple motors is encoded as expected.
(deftest test-encode-command
  (testing "Encode full command for multiple motors"
    (let [cmd {:motors [{:motor-number 0
                         :total-pulses 1000
                         :direction 1}
                        {:motor-number 1
                         :total-pulses 1200
                         :direction 0}]}
          buf (encode-command cmd)
          ;; Read out the ints from the ByteBuffer.
          ;; We expect: header: 2 motors, then the two motor parameter sequences.
          num-ints (/ (.limit buf) 4)
          ints (doall (for [_ (range num-ints)]
                        (.getInt buf)))]
      (is (= [0 1000 1 1 1200 0] ints)))))

;; Test the binary write function using a temporary file.
(deftest test-write-binary-message
  (testing "Write binary message to a temporary file"
    (let [cmd {:motors [{:motor-number 2
                         :total-pulses 1000
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
            (is (= 2 (.getInt buf-read)))        ; motor-number
            (is (= 1000 (.getInt buf-read)))     ; motor: total-pulses
            (is (= 1 (.getInt buf-read))))))
      (.delete tmp-file))))

;; Run the tests when using Babashka or nbb with the --test flag.
;; C-u C-c C-v C-f C-d
(run-tests)
;; => {:test 3, :pass 4, :fail 1, :error 0, :type :summary}
