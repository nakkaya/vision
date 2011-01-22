(ns detect-motion
  (:use [vision core] :reload-all))

(let [state (ref {:run true})]
  
  (defn start []
    (let [capture (capture-from-cam 0)]
      (dosync (alter state assoc
                     :run true
                     :prev (clone-image (query-frame capture))))
      (future
       (while (:run @state)
         (let [curr (query-frame capture)
               prev (:prev @state)
               processed (--> (abs-diff curr prev)
                              (convert-color :bgr-gray)
                              (smooth :gaussian 9 9 0 0)
                              (threshold 30 255 :binary))
               rects (with-contours [c [processed :external :chain-approx-none [0 0]]]
                       (bounding-rects c))
               display (clone-image curr)]

           (dosync (alter state assoc :prev (clone-image curr)))

           (doseq [[x y w h] rects]
             (rectangle display [x y] [(+ w x) (+ h y)] java.awt.Color/red 5))

           (view :motion display)

           (release [prev processed display])))
       (release capture))))

  (defn stop []
    (dosync (alter state assoc :run false))))
