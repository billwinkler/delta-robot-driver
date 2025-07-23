(ns delta-robot.tasks
  (:require
   [babashka.cli :as cli]
   [delta-robot.motion :as motion]
   [delta-robot.gripper :as gripper]
   [babashka.process :as p]
   [babashka.fs :as fs]
   [clojure.string :as str]
   [clojure.edn :as edn]))

(defn error-fn
  "Error-function called when parse-opts exception is caught"
  [{:keys [msg] :as data}]
  ;; when help flag is present, don't exit, let help text be printed
  (when-not (some #{"-h" "--help"} *command-line-args*)
    (when msg (println msg))
    (System/exit 1)))

(def help-spec {:help {:alias :h :desc "Display help for this task."}})

(def example-spec
  (merge help-spec
         {:num {:alias :n :coerce [:int] :desc "Coerces -n values into a vec of ints"}
          :foo {:require false :desc "Foo is not required"}
          :zee {:default 500 :desc "Default is 500"}}))

(defn example-task
  {:org.babashka/cli {:exec-args {:bar "function data"}
                      :spec example-spec
                      :args->opts [:arg-1]
                      :error-fn error-fn}}
  [{:keys [help] :as m}]
  (if help
    (println (cli/format-opts {:spec example-spec}))
    (do
      (println "for example:" m)
      (println "just an example, try -n, try an arg"))))

(def move-to-spec
  (merge help-spec
         {:x {:coerce :int
              :require true
              :desc "The X coordinate."
              :validate {:pred #(and (>= % -80) (<= % 80))
                         :ex-msg (fn [{:keys [value]}] (format "X must be between -80 and 80, got: '%s'" value))}}
          :y {:coerce :int
              :require true
              :desc "The Y coordinate."
              :validate {:pred #(and (>= % -80) (<= % 80))
                         :ex-msg (fn [{:keys [value]}] (format "Y must be between -80 and 80, got: '%s'" value))}}
          :z {:coerce :int
              :require true
              :desc "The Z coordinate."
              :validate {:pred #(and (>= % 190) (<= % 420))
                         :ex-msg (fn [{:keys [value]}] (format "Z must be between 190 and 420, got: '%s'" value))}}}))

(defn move-to
  "Moves effector to new xyz position"
  {:org.babashka/cli {:spec move-to-spec
                      :args->opts [:x :y :z]
                      :error-fn error-fn}}
  [{:keys [x y z help]}]
  (if help
    (println (cli/format-opts {:spec move-to-spec}))
    (motion/move-to x y z)))

(defn- open-gripper [m]
  (println "Opening gripper...")
  (gripper/open))

(defn- close-gripper [m]
  (println "Closing gripper...")
  (gripper/close))

(def gripper-set-spec {:mm {:coerce :int :require true :desc "The opening in millimeters."}})

(defn- set-gripper [{:keys [opts]}]
  (gripper/grip (:mm opts)))

(def gripper-dispatch-table
  [{:cmds ["open"]  :fn open-gripper}
   {:cmds ["close"] :fn close-gripper}
   {:cmds ["set"]   :fn set-gripper :spec gripper-set-spec :args->opts [:mm]}])

(defn gripper
  "Control the gripper. Subcommands: open, close, set <mm>"
  [& args]
  (cli/dispatch gripper-dispatch-table *command-line-args* {:error-fn error-fn}))

(defn home
  "Homes the robot"
  [m]
  (motion/home))

(def camera-spec
  (merge help-spec
         {:output-path {:type :string
                        :desc "The full path to save the captured image. Defaults to a timestamped filename in the current directory."}}))

(defn camera
  "Takes a picture using the camera"
  {:org.babashka/cli {:spec camera-spec
                      :args->opts [:output-path]
                      :error-fn error-fn}}
  [{:keys [output-path help]}]
  (if help
    (println (cli/format-opts {:spec camera-spec}))
    (let [path (or output-path
                   (let [timestamp (.format (java.time.LocalDateTime/now)
                                            (java.time.format.DateTimeFormatter/ofPattern "yyyy-MM-dd_HH-mm-ss"))]
                     (str "capture-" timestamp ".jpg")))]
      (p/shell "python/.venv/bin/python" "python/camera.py" path)
;;      (p/shell "python/.venv/bin/python" "python/pecan_detector_debug.py" path)
      )))

(def collect-data-spec
  (merge help-spec
         {:home-x {:coerce :int :default 0 :desc "Home X coordinate." :alias :x}
          :home-y {:coerce :int :default 0 :desc "Home Y coordinate." :alias :y}
          :z-height {:coerce :int :default 400 :desc "Constant Z height for movements." :alias :z}
          :max-offset {:coerce :int :default 50 :desc "Max random distance in mm for x and y."}
          :num-samples {:coerce :int :default 10 :desc "Number of images to collect." :alias :n}
          :data-dir {:type :string :default "pecan_training_data" :desc "Directory to store data."}}))

(defn- random-offset
  "Generates a random [x y] offset."
  [max-val]
  [(- (rand-int (* 2 max-val)) max-val)
   (- (rand-int (* 2 max-val)) max-val)])

(defn- capture-image!
  "Calls the python script to capture an image."
  [image-path]
  (let [result @(p/process ["python/.venv/bin/python" "python/camera.py" (str image-path)] {:err :inherit})]
    (when-not (zero? (:exit result))
      (println "Error capturing image."))))

(defn- collect-samples
  "Main loop to collect training data."
  [config]
  (let [{:keys [home-x home-y z-height num-samples max-offset images-dir timestamp]} config
        home-pos [home-x home-y z-height]]
    (loop [i 0
           labels []]
      (if (< i num-samples)
        (let [offset (random-offset max-offset)
              [offset-x offset-y] offset
              target-x (+ home-x offset-x)
              target-y (+ home-y offset-y)
              image-name (format "%s_sample_%04d.jpg" timestamp i)
              image-path (fs/path images-dir image-name)]

          (println (format "Sample %d/%d: offset %s" (inc i) num-samples offset))

          (motion/move-to target-x target-y z-height)
          (capture-image! image-path)
;;          (motion/move-to home-x home-y z-height)
;;          (Thread/sleep 1000)

          (recur (inc i) (conj labels {:image image-name
                                       :offset offset
                                       :home home-pos})))
        labels))))

(defn collect-data
  "Collects training data by taking pictures at random offsets."
  {:org.babashka/cli {:spec collect-data-spec
                      :error-fn error-fn}}
  [{:keys [help] :as opts}]
  (if help
    (println (cli/format-opts {:spec collect-data-spec}))
    (let [timestamp (.format (java.time.LocalDateTime/now)
                             (java.time.format.DateTimeFormatter/ofPattern "yyyy-MM-dd_HH-mm-ss"))
          config (assoc opts
                        :images-dir (fs/path (:data-dir opts) "images")
                        :timestamp timestamp)
          labels-path (fs/path (:data-dir opts) "labels.edn")]
      (println "Starting data collection with config:")
      (clojure.pprint/pprint (dissoc config :spec))
      (fs/create-dirs (:images-dir config))
      (motion/home)
      (println "Moving to intermediate safe height.")
      (motion/move-to 0 0 300)
      (println "Moving to data collection start position.")
      (motion/move-to (:home-x config) (:home-y config) (:z-height config))

      (let [existing-labels (if (fs/exists? labels-path)
                              (vec (edn/read-string (slurp (str labels-path))))
                              [])
            new-labels (collect-samples config)
            all-labels (into existing-labels new-labels)]
        (println "\nSaving" (count new-labels) "new labels to" labels-path)
        (spit (str labels-path) (with-out-str (clojure.pprint/pprint all-labels))))

      (println "Data collection complete."))))


