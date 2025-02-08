(ns delta-translator
  (:require [clojure.edn :as edn]
            [clojure.java.io :as io])
  (:import [java.nio ByteBuffer ByteOrder]))

;; Function to encode a single motor's command as a vector of 5 integers.
(defn encode-motor-command [motor]
  [(int (:total-pulses motor))
   (int (:target-freq motor))
   (int (:accel-pulses motor))
   (int (:decel-pulses motor))
   (int (:direction motor))])

;; Function to encode the complete command.
(defn encode-command [cmd]
  (let [motors (:motors cmd)
        num-motors (count motors)
        ;; Each motor contributes 5 ints; plus 1 int for the header (number of motors)
        total-ints (+ 1 (* num-motors 5))
        buf (ByteBuffer/allocate (* total-ints 4))]
    ;; Set the byte order (adjust if needed—here we use LITTLE_ENDIAN)
    (.order buf ByteOrder/LITTLE_ENDIAN)
    ;; Write the number of motors as the header
    (.putInt buf num-motors)
    ;; Write each motor’s parameters into the buffer
    (doseq [motor motors]
      (doseq [i (encode-motor-command motor)]
        (.putInt buf i)))
    ;; Flip the buffer so it’s ready for reading (or writing out)
    (.flip buf)
    buf))

;; Function to write the binary data to a given file path.
(defn write-binary-message [file-path buf]
  (with-open [out (io/output-stream file-path)]
    ;; Write exactly the number of bytes in the buffer.
    (let [arr (.array buf)
          len (.limit buf)]
      (.write out arr 0 len))))

;; The main function: read EDN from a file (or string), encode, and write out.
(defn -main [& args]
  (let [;; For example, read the EDN command from a file "command.edn"
        edn-string (slurp "command.edn")
        cmd (edn/read-string edn-string)
        buf (encode-command cmd)
        ;; Change this to your kernel module’s device file (or a test file)
        ;; out-file "/dev/delta_robot"
        out-file "/tmp/delta_robot"]
    (write-binary-message out-file buf)
    (println "Binary command sent to" out-file)))


(comment
  (let [cmd {:motors [{:total-pulses 1000
                       :target-freq 400
                       :accel-pulses 100
                       :decel-pulses 100
                       :direction 1}
                      {:total-pulses 1200
                       :target-freq 450
                       :accel-pulses 100
                       :decel-pulses 100
                       :direction 0}
                      {:total-pulses 1100
                       :target-freq 420
                       :accel-pulses 100
                       :decel-pulses 100
                       :direction 1}]}
        buf (encode-command cmd)
        out-file "/tmp/delta_robot"]
    (write-binary-message out-file buf))

  )

;; When running with bb, this -main function is invoked automatically.
(-main)
