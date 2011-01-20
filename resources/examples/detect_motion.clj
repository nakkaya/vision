(ns detect-motion
  (:use [vision core graphics] :reload-all))

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
                       (bounding-rects c))]

           (doseq [[x y w h] rects]
             (rectangle curr x y w h java.awt.Color/red 5))

           (dosync (alter state assoc :prev (clone-image curr)))

           (show-image :motion curr)

           (release-image prev)
           (release-image processed)))
       (release-capture capture))))

  (defn stop []
    (dosync (alter state assoc :run false))))
