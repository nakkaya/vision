(ns bounding-box
  (:use [vision core graphics] :reload-all))

(let [image (load-image "resources/samples/soccerfield.jpeg" :color)
      processed (--> (convert-color image :bgr-hsv)
                     (in-range-s [40 25 155 0] [50 70 255 0])
                     (smooth :gaussian 3 3 0 0))
      [[x y width height]] (with-contours [c [processed :external :chain-approx-none [0 0]]]
                             (bounding-rect c))]
  
  (rectangle image x y width height java.awt.Color/red 4)
  (show-image :result image)

  (release-image image)
  (release-image processed))
