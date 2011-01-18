(ns vision.core
  (:import (com.sun.jna Function Pointer)
           (com.sun.jna.ptr IntByReference FloatByReference)))

(System/setProperty "jna.library.path" "./resources/lib/")

(defmacro -->
  ([x] x)
  ([x form] `(let [img# ~x
                   next# (~(first form) img# ~@(next form))]
               (release-image img#)
               next#))
  ([x form & more] `(--> (--> ~x ~form) ~@more)))

(defmacro call [f arg & args]
  (if (empty? args)
    `(.invoke (Function/getFunction "vision" (name ~f)) (to-array ~arg))
    `(.invoke (Function/getFunction "vision" (name ~f)) ~arg (to-array ~@args))))

(defn- release-memory [p]
  (call :release_memory [p]))

(defn image-size
  "Get image width, height."
  [[p _]]
  (let [ref (call :image_size IntByReference [p])
        pointer (.getPointer ref)
        info (seq (.getIntArray pointer 0 2))]
    (release-memory ref)
    info))

(defn- pixels [p t]
  (let [ref (call :pixels IntByReference [p t])
        pointer (.getPointer ref)
        [width height] (image-size [p])
        pxs (.getIntArray pointer 0 (* width height))]
    (release-memory ref)
    pxs))

(defn- buffered-image [pxs t]
  (delay
   (let [[width height] (image-size [pxs])
         pxs (pixels pxs t)]
     (java.awt.image.BufferedImage.
      (. java.awt.image.ColorModel getRGBdefault)
      (java.awt.image.Raster/createPackedRaster
       (java.awt.image.DataBufferInt. pxs (* width height))
       width height width  (int-array [0xFF0000 0xFF00 0xFF 0xFF000000]) nil)
      false nil))))

(defn load-image
  "Loads an image from file
   c
    :color, :grayscale, :unchanged"
  [f c]
  {:pre  [(.exists (java.io.File. f))]}
  (let [ref (call :load_image Pointer [f (cond (= c :color) 1
                                               (= c :grayscale) 0
                                               (= c :unchanged) -1
                                               :default (throw (Exception. "Unknown Type.")))])
        type (cond (= c :color) 1
                   (= c :grayscale) 5
                   (= c :unchanged) 1)]
    [ref (buffered-image ref type) type]))

(defn release-image
  "Release allocated image."
  [[p _]]
  (call :release_image [p]))

(defn save-image
  "Saves an image to the file."
  [[p _] f]
  (call :save_image [p f]))

(defn capture-from-cam
  "Allocates CvCapture structure and  binds it to the video camera."
  [n]
  (call :capture_from_cam Pointer [n]))

(defn query-frame
  "Grabs and returns a frame from camera or file."
  [c]
  (let [ref (call :query_frame Pointer [c])]
    [ref (buffered-image ref 1) 1]))

(defn release-capture
  "Releases the CvCapture structure."
  [c]
  (call :release_capture [c]))

(defn hough-circles
  "Finds circles in grayscale image using some modification of Hough transform."
  [[i _] dp min_d p1 p2 min-r max-r]
  (if-let[ref (call :hough_circles FloatByReference [i dp min_d p1 p2 min-r max-r])]
    (let [pointer (.getPointer ref)
          count (.getFloat pointer 0)
          circles (partition 3 (seq (drop 1 (.getFloatArray pointer 0 (inc (* 3 count))))))]
      (release-memory ref)
      circles)
    []))

(defn bounding-rect
  "Calls cvFindContours on the image then iterates through seq returned, calling cvBoundingRect on each."
  [[i _]]
  (if-let[ref (call :bounding_rect IntByReference [i])]
    (let [pointer (.getPointer ref)
          count (.getInt pointer 0)
          rects (partition 4 (seq (drop 1 (.getIntArray pointer 0 (inc (* 4 count))))))]
      (release-memory ref)
      rects)
    []))

(defn match-template
  "Compares template against overlapped image regions.
   calculation
    :sqdiff, :sqdiff-normed, :ccorr, :ccorr-normed, :ccoeff, :ccoeff-normed"
  [[image _] [template _] calculation]
  (let [calculation (cond (= :sqdiff calculation) 1
                          (= :sqdiff-normed calculation) 2
                          (= :ccorr calculation) 3
                          (= :ccorr-normed calculation) 4
                          (= :ccoeff calculation) 5
                          (= :ccoeff-normed calculation) 6
                          :default (throw (Exception. "Unknown Calculation.")))
        ref (call :match_template FloatByReference [image template calculation])
        pointer (.getPointer ref)
        vals (.getIntArray pointer 0 6)]
    (release-memory ref)
    vals))

(defn match-shapes
  "Compares two shapes.
   calculation
    :i1, :i2, :i3"
  [[img1 _] [img2 _] calculation]
  (let [calculation (cond (= :i1 calculation) 1
                          (= :i2 calculation) 2
                          (= :i3 calculation) 3
                          :default (throw (Exception. "Unknown Calculation.")))]
    (call :match_shapes Double [img1 img2 calculation])))

(defn in-range-s
  "Checks that image elements lie between two scalars."
  [[p _] [s11 s12 s13 s14] [s21 s22 s23 s24]]
  (let [ref (call :in_range_s Pointer [p s11 s12 s13 s14 s21 s22 s23 s24])]
    [ref (buffered-image ref 2) 2]))

(defn convert-color
  "Converts image from one color space to another.
   m
    :rgb-hsv, :hsv-rgb, :bgr-hsv, :hsv-bgr, :bgr-gray, :gray-bgr"
  [[p _] m]
  (let [ref (call :convert_color Pointer [p (cond (= m :rgb-hsv) 1
                                                    (= m :hsv-rgb) 2
                                                    (= m :bgr-hsv) 3
                                                    (= m :hsv-bgr) 4
                                                    (= m :bgr-gray) 5
                                                    (= m :gray-bgr) 6
                                                    :default (throw (Exception. "Unknown Convertion.")))])
        type (cond (= m :rgb-hsv) 3
                   (= m :hsv-rgb) 4
                   (= m :bgr-hsv) 3
                   (= m :hsv-bgr) 1
                   (= m :bgr-gray) 5
                   (= m :gray-bgr) 1)]
    [ref (buffered-image ref type) type]))

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
  [[p _ t] m p1 p2 p3 p4]
  (let [ref (call :smooth Pointer [p (cond (= m :blur-no-scale) 1
                                             (= m :blur) 2
                                             (= m :gaussian) 3
                                             (= m :median) 4
                                             (= m :bilateral) 5
                                             :default (throw (Exception. "Unknown Convertion.")))
                                     p1 p2 p3 p4])]
    [ref (buffered-image ref t) t]))

(defn abs-diff
  "Calculates absolute difference between two images."
  [[p1 _ t] [p2 _ _]]
  (let [ref (call :abs_diff Pointer [p1 p2])]
    [ref (buffered-image ref t) t]))

(defn clone-image
  "Makes a full clone of the image (e.g., a duplicate full size image with its own matching ROI attached)"
  [[p _ t]]
  (let [ref (call :clone_image Pointer [p])]
    [ref (buffered-image ref t) t]))

(defn threshold
  "Applies fixed-level threshold to an image.
   threshold
     Threshold value.
   max-val
     Maximum value to use with :binary, :binary-inv, and :trunc thresholding types.
   thresholdType
     Thresholding type (see the cvThreshold)"
  [[p _] threshold max-val type]
  (let [ref (call :threshold Pointer [p (double threshold) (double max-val)
                                        (cond (= type :binary) 1
                                              (= type :binary-inv) 2
                                              (= type :trunc) 3
                                              (= type :to-zero) 4
                                              (= type :to-zero-inv) 5
                                              :default (throw (Exception. "Unknown Type.")))])]
    [ref (buffered-image ref 2) 2]))

(defn load-cascade
  "Load a HaarClassifierCascade."
  [f]
  {:pre  [(.exists (java.io.File. f))]}
  (if-let[ref (call :load_cascade Pointer [f])]
    ref nil))

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
  [[i _] cascade scale-factor min-neighbors flag [min-w min-h]]
  (if-let[ref (call :haar_detect_objects IntByReference
                      [i cascade (double scale-factor) min-neighbors 1 min-w min-h])]
    (let [pointer (.getPointer ref)
          count (.getInt pointer 0)
          rects (partition 4 (seq (drop 1 (.getIntArray pointer 0 (inc (* 4 count))))))]
      (release-memory ref)
      rects)
    []))
