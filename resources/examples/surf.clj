(ns surf
  (:use [vision core] :reload-all))

(let [obj-img (load-image "resources/samples/box.png" :grayscale)
      scene-img (load-image "resources/samples/box_in_scene.png" :grayscale)
      params (surf-params 1 500)
      obj-surf (extract-surf obj-img nil params)
      scene-surf (extract-surf scene-img nil params)
      location (surf-locate scene-surf obj-surf)]

  (doseq [[x y r] (surf-points obj-surf)] 
    (circle obj-img [x y] r java.awt.Color/blue 1))

  (doseq [[[x1 y1] [x2 y2]] (partition 2 1 (cons (last location) location))] 
    (line scene-img [x1 y1] [x2 y2] java.awt.Color/blue 1))
  
  (view :object obj-img)
  (view :scene scene-img)

  (release [obj-img scene-img obj-surf scene-surf]))
