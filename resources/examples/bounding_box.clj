(ns bounding-box
  (:use [vision core graphics] :reload-all))

(let [image (load-image "resources/samples/soccerfield.jpeg" :color)
      [[x y width height]] (--> (convert-color image :bgr-hsv)
                                (in-range-s [40 25 155 0] [50 70 255 0])
                                (smooth :gaussian 3 3 0 0)
                                (bounding-rect))]
  (rectangle image x y width height java.awt.Color/red 4)
  (view image))
