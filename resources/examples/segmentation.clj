(ns segmentation
  (:use [vision core] :reload-all))

(let [image (load-image "resources/samples/plate.jpeg" :color)
      processed (--> (convert-color image :bgr-gray)
                     (smooth :gaussian 3 3 0 0)
                     (threshold 30 255 :binary-inv)
                     (erode 1))
      cs (with-contours [c [processed :external :chain-approx-simple [0 0]]]
           (bounding-rects c))]

  (doseq [[x y w h] cs] 
    (rectangle image [x y] [(+ w x) (+ h y)] java.awt.Color/red 1))
  
  (view :result image)

  (release [image processed]))
