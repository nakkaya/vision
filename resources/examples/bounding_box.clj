(ns bounding-box
  (:use [vision core] :reload-all))

(let [image (load-image "resources/samples/soccerfield.jpeg" :color)
      processed (--> (convert-color image :bgr-hsv)
                     (in-range-s [40 25 155 0] [50 70 255 0])
                     (smooth :gaussian 3 3 0 0))
      [[x y width height]] (with-contours [c [processed :external :chain-approx-none [0 0]]]
                             (bounding-rects c))]
  
  (rectangle image [x y] [(+ width x) (+ height y)] java.awt.Color/red 4)
  (view :result image)

  (release [processed image]))
