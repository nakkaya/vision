(ns vision.core
 (:import (com.sun.jna Function Pointer)))
  
(System/setProperty "jna.library.path" "./resources/lib/")

(defmacro -->
  ([x] x)
  ([x form] `(let [img# ~x
                   next# (~(first form) img# ~@(next form))]
               (release-image img#)
               next#))
  ([x form & more] `(--> (--> ~x ~form) ~@more)))

(defn function [f]
 (Function/getFunction "vision" f))

(defn release-memory [p]
  (.invoke (function "release_memory") (to-array [p])))

(defn image-size [[p _]]
  (let [ref (.invoke (function "image_size") com.sun.jna.ptr.IntByReference (to-array [p]))
        pointer (.getPointer ref)
        info (seq (.getIntArray pointer 0 2))]
    (release-memory ref)
    info))

(defn pixels [p t]
  (let [ref (.invoke (function "pixels") com.sun.jna.ptr.IntByReference (to-array [p t]))
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

(defn load-image [f c]
  (let [ref (.invoke (function "load_image") Pointer (to-array [f (cond (= c :color) 1
                                                                        (= c :grayscale) 0
                                                                        (= c :unchanged) -1)]))]
    [ref (buffered-image ref 1) 1]))

(defn release-image [[p _]]
  (.invoke (function "release_image") (to-array [p])))

(defn save-image [[p _] f]
  (.invoke (function "save_image") (to-array [p f])))
(defn capture-from-cam [n]
  (.invoke (function "capture_from_cam") Pointer (to-array [n])))
  
(defn query-frame [c]
  (let [ref (.invoke (function "query_frame") Pointer (to-array [c]))]
    [ref (buffered-image ref 1) 1]))

(defn release-capture [c]
  (.invoke (function "release_capture") (to-array [c])))

(defn hough-circles [[i _] dp min_d p1 p2 min-r max-r]
  (if-let[ref (.invoke (function "hough_circles")
                       com.sun.jna.ptr.FloatByReference
                       (to-array [i dp min_d p1 p2 min-r max-r]))]
    (let [pointer (.getPointer ref)
          count (.getFloat pointer 0)
          circles (partition 3 (seq (drop 1 (.getFloatArray pointer 0 (inc (* 3 count))))))]
      (release-memory ref)
      circles)
    []))

(defn bounding-rect [[i _]]
  (if-let[ref (.invoke (function "bounding_rect") com.sun.jna.ptr.IntByReference (to-array [i]))]
    (let [pointer (.getPointer ref)
          count (.getInt pointer 0)
          rects (partition 4 (seq (drop 1 (.getIntArray pointer 0 (inc (* 4 count))))))]
      (release-memory ref)
      rects)
    []))

(defn match-template [[image _] [template _] calculation]
  (let [calculation (cond (= :sqdiff calculation) 1
                          (= :sqdiff-normed calculation) 2
                          (= :ccorr calculation) 3
                          (= :ccorr-normed calculation) 4
                          (= :ccoeff calculation) 5
                          (= :ccoeff-normed calculation) 6)
        ref (.invoke (function "match_template")
                     com.sun.jna.ptr.FloatByReference
                     (to-array [image template calculation]))
        pointer (.getPointer ref)
        vals (.getIntArray pointer 0 6)]
    (release-memory ref)
    vals))

(defn match-shapes [[img1 _] [img2 _] calculation]
  (let [calculation (cond (= :i1 calculation) 1
                          (= :i2 calculation) 2
                          (= :i3 calculation) 3)]
    (.invoke (function "match_shapes") Double (to-array [img1 img2 calculation]))))

(defn in-range-s [[p _] [s11 s12 s13 s14] [s21 s22 s23 s24]]
  (let [ref (.invoke (function "in_range_s") Pointer (to-array [p s11 s12 s13 s14 s21 s22 s23 s24]))]
    [ref (buffered-image ref 2) 2]))

(defn convert-color [[p _] m]
  (let [ref (.invoke (function "convert_color") Pointer
                     (to-array [p (cond (= m :rgb-hsv) 1
                                        (= m :hsv-rgb) 2
                                        (= m :bgr-hsv) 3
                                        (= m :hsv-bgr) 4
                                        :default (throw (Exception. "Unknown Convertion.")))]))
        type (cond (= m :rgb-hsv) 3
                   (= m :hsv-rgb) 4
                   (= m :bgr-hsv) 3
                   (= m :hsv-bgr) 1)]
    [ref (buffered-image ref type) type]))

(defn smooth [[p _ t] m p1 p2 p3 p4]
  (let [ref (.invoke (function "smooth") Pointer
                     (to-array [p (cond (= m :blur-no-scale) 1
                                        (= m :blur) 2
                                        (= m :gaussian) 3
                                        (= m :median) 4
                                        (= m :bilateral) 5
                                        :default (throw (Exception. "Unknown Convertion.")))
                                p1 p2 p3 p4]))]
    [ref (buffered-image ref t) t]))

(defn abs-diff [[p1 _ t] [p2 _ _]]
  (let [ref (.invoke (function "abs_diff") Pointer (to-array [p1 p2]))]
    [ref (buffered-image ref t) t]))

(defn clone-image [[p _ t]]
  (let [ref (.invoke (function "clone_image") Pointer (to-array [p]))]
    [ref (buffered-image ref t) t]))
