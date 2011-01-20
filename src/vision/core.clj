(ns vision.core
  (:use [clojure.contrib.def :only [defmacro-]])
  (:import (com.sun.jna Function Pointer)
           (com.sun.jna.ptr IntByReference FloatByReference)))

(System/setProperty "jna.library.path" "./resources/lib/")


(defrecord IplImage [#^Pointer pointer
                     #^clojure.lang.Keyword color-space
                     #^java.awt.image.BufferedImage buffered-image])

(defmacro -->
  "Threads images through the forms. Passing images from call to call and relasing
   all but the last image."
  ([x] x)
  ([x form] `(let [img# ~x
                   next# (~(first form) img# ~@(next form))]
               (release-image img#)
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
        (release-memory ref#)))))

(defn- release-memory [p]
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
         pxs (pixels pxs t)]
     (java.awt.image.BufferedImage.
      (. java.awt.image.ColorModel getRGBdefault)
      (java.awt.image.Raster/createPackedRaster
       (java.awt.image.DataBufferInt. pxs (* width height))
       width height width  (int-array [0xFF0000 0xFF00 0xFF 0xFF000000]) nil)
      false nil))))

(defn- ipl-image [ref cs]
  (let [type (cond (= cs :color) 1
                   (= cs :unchanged) 1
                   (= cs :binary) 2
                   (= cs :grayscale) 5
                   (= cs :rgb-hsv) 3
                   (= cs :hsv-rgb) 4
                   (= cs :bgr-hsv) 3
                   (= cs :hsv-bgr) 1
                   (= cs :bgr-gray) 5
                   (= cs :gray-bgr) 1
                   :default cs)]
    (IplImage. ref type (buffered-image ref type))))

(defn load-image
  "Loads an image from file
   c
    :color, :grayscale, :unchanged"
  [f c]
  {:pre  [(.exists (java.io.File. f))]}
  (let [ref (call :load_image Pointer [f (cond (= c :color) 1
                                               (= c :grayscale) 0
                                               (= c :unchanged) -1
                                               :default (throw (Exception. "Unknown Type.")))])]
    (ipl-image ref c)))

(defn release-image
  "Release allocated image."
  [{p :pointer}]
  (call :release_image [p]))

(defn save-image
  "Saves an image to the file."
  [{p :pointer} f]
  (call :save_image [p f]))

(defn capture-from-cam
  "Allocates CvCapture structure and  binds it to the video camera."
  [n]
  (call :capture_from_cam Pointer [n]))

(defn capture-from-file
  "Initializes capturing a video from a file."
  [f]
  {:pre  [(.exists (java.io.File. f))]}
  (call :capture_from_file Pointer [f]))

(defn query-frame
  "Grabs and returns a frame from camera or file."
  [c]
  (ipl-image (call :query_frame Pointer [c]) :color))

(defn release-capture
  "Releases the CvCapture structure."
  [c]
  (call :release_capture [c]))

(defn hough-circles
  "Finds circles in grayscale image using some modification of Hough transform."
  [{i :pointer} dp min_d p1 p2 min-r max-r]
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
                                                   :default (throw (Exception. "Unknown Convertion.")))]) m))

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
     Thresholding type (see the cvThreshold)"
  [{p :pointer} threshold max-val type]
  (ipl-image (call :threshold Pointer [p (double threshold) (double max-val)
                                       (cond (= type :binary) 1
                                             (= type :binary-inv) 2
                                             (= type :trunc) 3
                                             (= type :to-zero) 4
                                             (= type :to-zero-inv) 5
                                             :default (throw (Exception. "Unknown Type.")))]) 2))

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
   Minimum window size."
  [{i :pointer} cascade scale-factor min-neighbors flag [min-w min-h]]
  (with-pointer [p (call :haar_detect_objects IntByReference
                         [i cascade (double scale-factor) min-neighbors 1 min-w min-h])]
    (let [count (.getInt p 0)]
      (partition 4 (seq (drop 1 (.getIntArray p 0 (inc (* 4 count)))))))))

(defn find-contours [{image :pointer} mode method [x y]]
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
    (call :find_contours Pointer [image mode method x y])))

(defn release-contours [contours]
  (call :release_contours [contours]))

(defmacro with-contours
  [bindings & body]
  {:pre  [(vector? bindings) (even? (count bindings))]}
  (cond
   (= (count bindings) 0) `(do ~@body)
   (symbol? (bindings 0)) `(let [~(first bindings) (find-contours ~@(second bindings))]
                             (try
                               (with-contours ~(subvec bindings 2) ~@body)
                               (finally
                                (release-contours ~(bindings 0)))))
   :else (throw (IllegalArgumentException.
                 "with-contours only allows Symbols in bindings"))))

(defn bounding-rects
  "Returns the up-right bounding rectangles for contours."
  [contours]
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
