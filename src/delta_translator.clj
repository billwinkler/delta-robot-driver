(ns delta-translator
  (:require [clojure.edn :as edn]
            [clojure.java.io :as io])
  (:import [java.nio ByteBuffer ByteOrder]))

;; Function to encode a single motor's command as a vector of 3 integers.
(defn encode-motor-command [motor]
  [(int (:motor-number motor))
   (int (:total-pulses motor))
   (int (:direction motor))])

;; Function to encode the complete command.
(defn encode-command [cmd]
  (let [motors (:motors cmd)
        num-motors (count motors)
        ;; Each motor contributes 3 ints
        total-ints (* num-motors 3)
        buf (ByteBuffer/allocate (* total-ints 4))]
    ;; Set the byte order (adjust if needed—here we use LITTLE_ENDIAN)
    (.order buf ByteOrder/LITTLE_ENDIAN)
    ;; Write the number of motors as the header
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
  (let [cmd {:motors [{:motor-number 1
                       :total-pulses 1200
                       :direction 0}
                      {:motor-number 0
                       :total-pulses 1200
                       :direction 0}
                      {:motor-number 2
                       :total-pulses 1200
                       :direction 0}]}
        buf (encode-command cmd)
        out-file "/tmp/delta_robot"]
    (write-binary-message out-file buf))

  )

;; When running with bb, this -main function is invoked automatically.
(-main)
