(ns vision.core
  (:use [clojure.contrib.def :only [defmacro-]])
  (:import (com.sun.jna Function Pointer)
           (com.sun.jna.ptr ByReference IntByReference FloatByReference)
           (java.awt.image BufferedImage DirectColorModel Raster DataBufferInt)))

(defrecord IplImage [#^Pointer pointer
                     #^clojure.lang.Keyword color-space
                     #^java.awt.image.BufferedImage buffered-image])
(defrecord Capture [#^Pointer pointer source])
(defrecord Contours [#^Pointer pointer])
(defrecord VideoWriter [#^Pointer pointer])
(defrecord Surf [#^Pointer pointer])
(defrecord CamShift [#^Pointer pointer])

(defmulti release
  "Release resource."
  class)

(defmethod release clojure.lang.PersistentVector [xs]
           (doseq [x xs] 
             (release x)))

(defmacro -->
  "Threads images through the forms. Passing images from call to call and relasing
   all but the last image."
  ([x] x)
  ([x form] `(let [img# ~x
                   next# (~(first form) img# ~@(next form))]
               (release img#)
               next#))
  ([x form & more] `(--> (--> ~x ~form) ~@more)))

(defmacro- call [f arg & args]
  (if (empty? args)
    `(.invoke (Function/getFunction "vision" (name ~f)) (to-array ~arg))
    `(.invoke (Function/getFunction "vision" (name ~f)) ~arg (to-array ~@args))))

(defmacro- with-pointer
  [binding & body]
  {:pre  [(vector? binding) (= 2 (count binding))]}
  `(if-let [ref# ~(second binding)]
     (try
       (let [~(first binding) (.getPointer ref#)]
         ~@body)
       (finally
        (release ref#)))))

(defmethod release ByReference [p]
           (call :release_memory [p]))

(defmulti image-size
  "Get image width, height."
  class)

(defmethod image-size Pointer [p]
           (with-pointer [p (call :image_size IntByReference [p])]
             (seq (.getIntArray p 0 2))))

(defmethod image-size IplImage [{p :pointer}]
           (with-pointer [p (call :image_size IntByReference [p])]
             (seq (.getIntArray p 0 2))))

(defn- pixels [p t]
  (with-pointer [ptr (call :pixels IntByReference [p t])]
    (let [[width height] (image-size p)]
      (.getIntArray ptr 0 (* width height)))))

(defn- buffered-image [pxs t]
  (delay
   (let [[width height] (image-size pxs)
         pxs (pixels pxs t)
         masks (int-array [0xFF0000 0xFF00 0xFF])]
     (BufferedImage.
      (DirectColorModel. 32 (first masks) (second masks) (last masks))
      (Raster/createPackedRaster
       (DataBufferInt. pxs (* width height)) width height width  masks nil)
      false nil))))

(defn- ipl-image [ref cs]
  (let [type (cond (= cs :bgr) 1
                   (= cs :binary) 2
                   (= cs :hsv) 3
                   (= cs :rgb) 4
                   (= cs :grayscale) 5
                   :default (throw (Exception. "Unknown Color Space.")))]
    (IplImage. ref cs (buffered-image ref type))))

(defn load-image
  "Loads an image from file
   c
    :color, :grayscale, :unchanged"
  [f c]
  {:pre  [(.exists (java.io.File. f))]}
  (let [ref (call :load_image Pointer [f (cond (= c :color) 1
                                               (= c :grayscale) 0
                                               :default (throw (Exception. "Unknown Type.")))])]
    (ipl-image ref (if (= c :color) :bgr :grayscale))))

(defmethod release IplImage [img]
           (call :release_image [(:pointer img)]))

(defn save-image
  "Saves an image to the file."
  [img f]
  {:pre [(instance? IplImage img) (instance? String f)]}
  (call :save_image [(:pointer img) f]))

(defn capture-from-cam
  "Allocates CvCapture structure and  binds it to the video camera."
  [n]
  (Capture. (call :capture_from_cam Pointer [n]) n))

(defn capture-from-file
  "Initializes capturing a video from a file."
  [f]
  {:pre  [(.exists (java.io.File. f))]}
  (Capture. (call :capture_from_file Pointer [f]) f))

(defn get-capture-property [{c :pointer} p]
  (call :get_capture_property Double [c (cond (= p :pos-msec) 1
                                              (= p :pos-frames) 2
                                              (= p :pos-avi-ratio) 3
                                              (= p :frame-width) 4
                                              (= p :frame-height) 5
                                              (= p :fps) 6
                                              (= p :fourcc) 7
                                              (= p :frame-count) 8
                                              (= p :brightness) 9
                                              (= p :contrast) 10
                                              (= p :saturation) 11
                                              (= p :hue) 12
                                              :default (throw (Exception. "Unknown Property.")))]))

(defn set-capture-property [{c :pointer} p v]
  (call :set_capture_property Integer [c (cond (= p :pos-msec) 1
                                               (= p :pos-frames) 2
                                               (= p :pos-avi-ratio) 3
                                               (= p :frame-width) 4
                                               (= p :frame-height) 5
                                               (= p :fps) 6
                                               (= p :fourcc) 7
                                               :default (throw (Exception. "Unknown Property.")))
                                       (double v)]))

(defn query-frame
  "Grabs and returns a frame from camera or file."
  [{c :pointer}]
  (ipl-image (call :query_frame Pointer [c]) :bgr))

(defn grab-frame
  "Grabs frame from camera or AVI."
  [{c :pointer}]
  (call :grab_frame Integer [c]))

(defmethod release Capture [cap]
           (call :release_capture [(:pointer cap)]))

(defn hough-circles
  "Finds circles in grayscale image using some modification of Hough transform."
  [{i :pointer cs :color-space} dp min_d p1 p2 min-r max-r]
  {:pre [(some true? (map #(= % cs) [:binary :grayscale]))]}
  (with-pointer [p (call :hough_circles FloatByReference [i dp min_d p1 p2 min-r max-r])]
    (let [count (.getFloat p 0)]
      (partition 3 (seq (drop 1 (.getFloatArray p 0 (inc (* 3 count)))))))))

(defn match-template
  "Compares template against overlapped image regions.
   calculation
    :sqdiff, :sqdiff-normed, :ccorr, :ccorr-normed, :ccoeff, :ccoeff-normed"
  [{image :pointer} {template :pointer} calculation]
  (with-pointer [p (call :match_template FloatByReference
                         [image template (cond (= :sqdiff calculation) 1
                                               (= :sqdiff-normed calculation) 2
                                               (= :ccorr calculation) 3
                                               (= :ccorr-normed calculation) 4
                                               (= :ccoeff calculation) 5
                                               (= :ccoeff-normed calculation) 6
                                               :default (throw (Exception. "Unknown Calculation.")))])]
    (.getIntArray p 0 6)))

(defn match-shapes
  "Compares two shapes.
   calculation
    :i1, :i2, :i3"
  [{img1 :pointer} {img2 :pointer} calculation]
  (let [calculation (cond (= :i1 calculation) 1
                          (= :i2 calculation) 2
                          (= :i3 calculation) 3
                          :default (throw (Exception. "Unknown Calculation.")))]
    (call :match_shapes Double [img1 img2 calculation])))

(defn in-range-s
  "Checks that image elements lie between two scalars."
  [{p :pointer} [s11 s12 s13 s14] [s21 s22 s23 s24]]
  (ipl-image (call :in_range_s Pointer [p s11 s12 s13 s14 s21 s22 s23 s24]) :binary))

(defn convert-color
  "Converts image from one color space to another.
   m
    :rgb-hsv, :hsv-rgb, :bgr-hsv, :hsv-bgr, :bgr-gray, :gray-bgr"
  [{p :pointer} m]
  (ipl-image (call :convert_color Pointer [p (cond (= m :rgb-hsv) 1
                                                   (= m :hsv-rgb) 2
                                                   (= m :bgr-hsv) 3
                                                   (= m :hsv-bgr) 4
                                                   (= m :bgr-gray) 5
                                                   (= m :gray-bgr) 6
                                                   :default (throw (Exception. "Unknown Convertion.")))])
             (cond (= m :rgb-hsv) :hsv
                   (= m :hsv-rgb) :rgb
                   (= m :bgr-hsv) :hsv
                   (= m :hsv-bgr) :bgr
                   (= m :bgr-gray) :grayscale
                   (= m :gray-bgr) :bgr)))

(defn smooth
  "Smooths the image in one of several ways.
   m
    Type of the smoothing, :blur-no-scale, :blur, :gaussian, :median, :bilateral
   p1
     The aperture width.
   p2
     The aperture height.
   p3
     Gaussian"
  [{p :pointer t :color-space} m p1 p2 p3 p4]
  (ipl-image (call :smooth Pointer [p (cond (= m :blur-no-scale) 1
                                            (= m :blur) 2
                                            (= m :gaussian) 3
                                            (= m :median) 4
                                            (= m :bilateral) 5
                                            :default (throw (Exception. "Unknown Convertion.")))
                                    p1 p2 p3 p4]) t))

(defn abs-diff
  "Calculates absolute difference between two images."
  [{p1 :pointer t :color-space} {p2 :pointer}]
  (ipl-image (call :abs_diff Pointer [p1 p2]) t))

(defn clone-image
  "Makes a full clone of the image (e.g., a duplicate full size image with its own matching ROI attached)"
  [{p :pointer t :color-space}]
  (ipl-image (call :clone_image Pointer [p]) t))

(defn threshold
  "Applies fixed-level threshold to an image.
   threshold
     Threshold value.
   max-val
     Maximum value to use with :binary, :binary-inv, and :trunc thresholding types.
   thresholdType
     Thresholding type (see the cvThreshold)
   if you don't provide threshold it will automatically calculated using otsu method."
  ([image max-val type]
     (threshold image (double -1) max-val type))
  ([{p :pointer cs :color-space} threshold max-val type]
     {:pre [(= cs :grayscale)]}
     (ipl-image (call :threshold Pointer [p (double threshold) (double max-val)
                                          (cond (= type :binary) 1
                                                (= type :binary-inv) 2
                                                (= type :trunc) 3
                                                (= type :to-zero) 4
                                                (= type :to-zero-inv) 5
                                                :default (throw (Exception. "Unknown Type.")))]) :binary)))

(defn load-cascade
  "Load a HaarClassifierCascade."
  [f]
  {:pre  [(.exists (java.io.File. f))]}
  (call :load_cascade Pointer [f]))

(defn haar-detect-objects
  "Detects objects in the image.
  [i _]
   Image to detect objects in.
  cascade
   Haar classifier cascade in internal representation.
  scale-factor
   The factor by which the search window is scaled between the subsequent scans.
  min-neighbors
   Minimum number (minus 1) of neighbor rectangles that makes up an object.
  flags
   Mode of operation. Currently the only flag that may be specified is :haar-do-canny-prunning.
  [min-w min-h]
   Minimum window size.
  [max-w max-h]
   Maximum window size."
  [{i :pointer} cascade scale-factor min-neighbors flag [min-w min-h] [max-w max-h]]
  (with-pointer [p (call :haar_detect_objects IntByReference
                         [i cascade (double scale-factor) min-neighbors 1 min-w min-h max-w max-h])]
    (let [count (.getInt p 0)]
      (partition 4 (seq (drop 1 (.getIntArray p 0 (inc (* 4 count)))))))))

(defn find-contours
  [{image :pointer cs :color-space} mode method [x y]]
  {:pre [(some true? (map #(= % cs) [:binary :grayscale]))]}
  (let [mode (cond (= mode :external) 1
                   (= mode :list) 2
                   (= mode :ccomp) 3
                   (= mode :tree) 4
                   :default (throw (Exception. "Unknown Mode.")))
        method (cond (= method :chain-code) 1
                     (= method :chain-approx-none) 2
                     (= method :chain-approx-simple) 3
                     (= method :chain-approx-tc89-l1) 4
                     (= method :chain-approx-tc89-kcos) 5
                     (= method :link-runs) 6
                     :default (throw (Exception. "Unknown Method.")))]
    (Contours. (call :find_contours Pointer [image mode method x y]))))

(defmethod release Contours [contours]
           (call :release_contours [(:pointer contours)]))

(defmacro with-contours
  [bindings & body]
  {:pre  [(vector? bindings) (even? (count bindings))]}
  (cond
   (= (count bindings) 0) `(do ~@body)
   (symbol? (bindings 0)) `(let [~(first bindings) (find-contours ~@(second bindings))]
                             (try
                               (with-contours ~(subvec bindings 2) ~@body)
                               (finally
                                (release ~(bindings 0)))))
   :else (throw (IllegalArgumentException.
                 "with-contours only allows Symbols in bindings"))))

(defn bounding-rects
  "Returns the up-right bounding rectangles for contours."
  [{contours :pointer}]
  (with-pointer [p (call :bounding_rects IntByReference [contours])]
    (let [count (.getInt p 0)]
      (partition 4 (seq (drop 1 (.getIntArray p 0 (inc (* 4 count)))))))))

(defn erode
  "Erodes image by using arbitrary structuring element"
  [{p :pointer t :color-space} iterations]
  (ipl-image (call :erode Pointer [p iterations]) t))

(defn dilate
  "Dilates image by using arbitrary structuring element"
  [{p :pointer t :color-space} iterations]
  (ipl-image (call :dilate Pointer [p iterations]) t))

(defn canny
  "Implements Canny algorithm for edge detection."
  [{p :pointer} threshold1 threshold2 aperture-size]
  (ipl-image (call :canny Pointer [p threshold1 threshold2 aperture-size]) :binary))

(defn hough-lines
  "Finds lines in grayscale image using a Hough transform."
  [{i :pointer cs :color-space} method rho theta threshold param1 param2]
  {:pre [(= cs :binary)]}
  (let [m (cond (= method :standard) 1
                (= method :probabilistic) 2
                (= method :multi-scale) 3
                :default (throw (Exception. "Unknown Method.")))]
    (with-pointer [p (call :hough_lines FloatByReference
                           [i m (double rho) (double theta) threshold (double param1) (double param2)])]
      (let [count (.getFloat p 0)]
        (if (or (= method :standard)
                (= method :multi-scale))
          (partition 2 (seq (drop 1 (.getFloatArray p 0 (inc (* 2 count))))))
          (partition 4 (seq (drop 1 (.getFloatArray p 0 (inc (* 4 count)))))))))))

(defn copy-region
  "Make a copy of the region from image"
  [{p :pointer cs :color-space} x y width height]
  (ipl-image (call :copy_region Pointer [p x y width height]) cs))

(defn rotate-image
  "Rotate image by a degree."
  [{p :pointer cs :color-space} a]
  (ipl-image (call :rotate_image Pointer [p (float a)]) cs))

(defn scale-image
  "Rotate image by a factor."
  [{p :pointer cs :color-space} s]
  (ipl-image (call :scale_image Pointer [p (double s)]) cs))

(defn video-writer
  "Creates video file writer."
  [f cc fps w h color?]
  (VideoWriter. (call :video_writer Pointer [f cc fps w h (if color? 1 0)])))

(defmethod release VideoWriter [{p :pointer}]
           (call :release_video_writer [p]))

(defn write-frame
  "Writes a frame to video file."
  [{p :pointer} {img :pointer}]
  (call :write_frame Integer [p img]))

(defn undistort-map
  "Computes an undistortion map."
  [intrinsic distortion [width height]]
  (call :undistort_map_from_file Pointer [intrinsic distortion width height]))

(defn remap
  "Applies a generic geometrical transformation to the image."
  [{p :pointer cs :color-space} distortion-map]
  (ipl-image (call :remap Pointer [p distortion-map]) cs))

(defn surf-params
  ([extended threshold]
     (surf-params extended threshold 3 4))
  ([extended threshold nOctaves nOctaveLayers]
     [extended threshold nOctaves nOctaveLayers]))

(defn extract-surf [{img :pointer cs :color-space} mask [extended threshold nOctaves nOctaveLayers]]
  {:pre [(= cs :grayscale)]}
  (Surf. (call :extract_surf Pointer [img mask extended (double threshold) nOctaves nOctaveLayers])))

(defn surf-points [{p :pointer}]
  (with-pointer [p (call :surf_points IntByReference [p])]
    (let [count (.getInt p 0)]
      (partition 3 (seq (drop 1 (.getIntArray p 0 (inc (* 3 count)))))))))

(defn surf-locate [{scene :pointer} {obj :pointer}]
  (with-pointer [p (call :locatePlanarObject IntByReference [obj scene])]
    (partition 2 (seq (.getIntArray p 0 8)))))

(defmethod release Surf [s]
           (call :release_surf [(:pointer s)]))

(defn pyr-down
  "Downsamples an image."
  [{p :pointer t :color-space}]
  (ipl-image (call :pyr_down Pointer [p]) t))

(defn camshift-init
  "Create a camshift tracked object from a region in image."
  [{img :pointer} [x y w h] vmin vmax smin]
  (CamShift. (call :camshift_init Pointer [img x y w h vmin vmax smin])))

(defn cam-shift-back-project-image
  [{c :pointer}]
  (ipl-image (call :cam_shift_back_project_image Pointer [c]) :binary))

(defn camshift-track
  "Given an image and tracked object, return box position."
  [{img :pointer} {str :pointer}]
  (with-pointer [p (call :camshift_track FloatByReference [img str])]
    (seq (.getFloatArray p 0 5))))

(defmethod release CamShift [s]
           (call :release_camshift [(:pointer s)]))

(defn max-rect
  "Given an image and tracked object, return box position."
  [[x1 y1 w1 h1] [x2 y2 w2 h2]]
  (with-pointer [p (call :max_rect IntByReference [x1 y1 w1 h1 x2 y2 w2 h2])]
    (seq (.getIntArray p 0 4))))

(defn good-features-to-track
  "Determines strong corners on an image."
  [{i :pointer} max-count quality min-distance win-size]
  (with-pointer [p (call :good_features_to_track FloatByReference
                         [i max-count (double quality) (double min-distance) win-size])]
    (let [count (.getFloat p 0)]
      (partition 2 (seq (drop 1 (.getFloatArray p 0 (inc (* 2 count)))))))))

(defn calc-optical-flow-pyr-lk
  "Calculates the optical flow for a sparse feature set using the iterative Lucas-Kanade method with pyramids."
  [{a :pointer} {b :pointer} points win-size level]
  (with-pointer [p (call :calc_optical_flow_pyr_lk FloatByReference
                         [a b (count points) (float-array (flatten points)) win-size level])]
    (let [s (partition 3 (seq (.getFloatArray p 0 (* 3 (count points)))))]
      (map #(take 2 %) (filter #(not= 0 (last %)) s)))))

;;
;; GUI Calls
;;

(defn ellipse-box
  "Draws a simple or thick elliptic arc."
  [{p :pointer} [x y w h a] c thickness]
  (call :ellipse_box [p x y w h a (.getRed c) (.getGreen c) (.getBlue c) thickness]))

(defn circle
  "Draws simple, thick or filled circle."
  [{p :pointer} [x y] r c thickness]
  (call :circle [p (int x) (int y) (int r) (.getRed c) (.getGreen c) (.getBlue c) thickness]))

(defn line
  "Draws simple or thick line segment."
  [{p :pointer} [x1 y1] [x2 y2] c thickness]
  (call :line [p (int x1) (int y1) (int x2) (int y2) (.getRed c) (.getGreen c) (.getBlue c) thickness]))

(defn poly-line
  "Draws a closed polygon."
  [image pts color thickness]
  (doseq [[p1 p2] (partition 2 1 [(first pts)] pts)]
    (line image p1 p2 color thickness)))

(defn rectangle
  "Draws simple or thick rectangle."
  [{p :pointer} [x1 y1] [x2 y2] c thickness]
  (call :rectangle [p (int x1) (int y1) (int x2) (int y2) (.getRed c) (.getGreen c) (.getBlue c) thickness]))

(defn contours
  "Draws contour outlines in the image"
  [{ip :pointer} {cp :pointer} c1 c2 level thickness]
  (call :contours [ip cp
                   (.getRed c1) (.getGreen c1) (.getBlue c1)
                   (.getRed c2) (.getGreen c2) (.getBlue c2)
                   thickness]))

(def *frames* (ref {}))

(defn- image-panel [f]
  (let [p (proxy [javax.swing.JPanel] []
            (paintComponent [g] (.drawImage g (-> @*frames* f :image) 0 0 this))
            (getPreferredSize [] (java.awt.Dimension.
                                  (.getWidth (-> @*frames* f :image))
                                  (.getHeight (-> @*frames* f :image)))))]
    (dosync (alter *frames* assoc-in [f :panel] p))
    (add-watch *frames* (str f) (fn [k r o n] (.repaint p)))
    p))

(defn- window-adapter [f]
  (proxy [java.awt.event.WindowAdapter] [] 
    (windowClosing [e] (dosync (alter *frames* dissoc f)))))

(defn- image-frame [f]
  (let [frame (doto (javax.swing.JFrame. (str f))
                (.add (image-panel f))
                (.setAlwaysOnTop true)
                (.pack)
                (.addWindowListener (window-adapter f))
                (.setVisible true))]
    (dosync (alter *frames* assoc-in [f :frame] frame))))

(defn view
  "Displays the image in a frame."
  [f {i :buffered-image}]
  (dosync (alter *frames* assoc-in [f :image] @i))  
  (when (nil? (-> @*frames* f :frame))
    (image-frame f)))

(defn move-window
  "Sets the position of the window."
  [f x y]
  (if-let[f (-> @*frames* f :frame)]
    (.setLocation f x y)))

(defn hide-window
  "Hide frame."
  [f]
  (if-let[f (-> @*frames* f :frame)]
    (doto f
      (.setAlwaysOnTop false)
      (.hide))))
