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
               rect (--> (abs-diff curr prev)
                         (convert-color :bgr-gray)
                         (smooth :gaussian 9 9 0 0)
                         (threshold 30 255 :binary)
                         (bounding-rect))]

           (doseq [[x y w h] rect]
             (rectangle curr x y w h java.awt.Color/red 5))

           (dosync (alter state assoc :prev (clone-image curr)))
           (show-image :motion curr)
           (release-image prev)))
       (release-capture capture))))

  (defn stop []
    (dosync (alter state assoc :run false))))
