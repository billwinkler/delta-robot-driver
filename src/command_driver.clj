(ns command-driver
  (:require [babashka.process :refer [sh]]
            [clojure.java.io :as io])
  (:import [java.io ByteArrayOutputStream DataOutputStream]))


(def raspberry-pi-host "raspberrypi.local") 
(def device-path "/dev/delta_robot")

(defn serialize-int [output-stream value]
  "Writes a 4-byte little-endian integer to the output stream."
  (let [bytes (byte-array 4)]
    (doseq [i (range 4)]
      (aset bytes i (unchecked-byte (bit-and (bit-shift-right value (* i 8)) 0xFF))))
    (.write output-stream bytes)))

(defn serialize-command [cmd]
  "Serializes a single motor command into a binary format."
  (let [{:keys [motor-number total-pulses direction]} cmd
        output-stream (ByteArrayOutputStream.)
        data-stream (DataOutputStream. output-stream)]
    (serialize-int data-stream motor-number)
    (serialize-int data-stream total-pulses)
    (serialize-int data-stream direction)
    (.toByteArray output-stream)))

(defn send-commands [commands]
  "Sends a batch of motor commands to the Raspberry Pi."
  (let [binary-data (byte-array (mapcat serialize-command commands))
        temp-file "/tmp/delta_command.bin"]
    
    ;; Write the binary data to a temporary file
    (with-open [out (io/output-stream temp-file)]
      (.write out binary-data))

    ;; Copy the binary file to the Raspberry Pi
    (let [scp-result (sh "scp" temp-file (str raspberry-pi-host ":/tmp/delta_command.bin"))
          ssh-result (sh "ssh" raspberry-pi-host (str "cat /tmp/delta_command.bin > " device-path))]

      (if (zero? (:exit ssh-result))
        (println "Command sent successfully!")
        (println "Failed to send command:" (:err ssh-result))))))

;; Example usage
(def motor-commands
)

(comment
  (let [pulse0 6000
        pulse1 5000
        pulse2 4000
        up [{:motor-number 0 :total-pulses pulse0 :direction 0}
            {:motor-number 1 :total-pulses pulse1 :direction 0}
            {:motor-number 2 :total-pulses pulse2 :direction 0}]
        dn [{:motor-number 0 :total-pulses pulse0 :direction 1}
            {:motor-number 1 :total-pulses pulse1 :direction 1}
            {:motor-number 2 :total-pulses pulse2 :direction 1}]]
    (dotimes [n 3]
      (do (send-commands up)
          (send-commands dn))))

  )
;; => nil
