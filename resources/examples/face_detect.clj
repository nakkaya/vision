(ns face-detect
  (:use [vision core graphics] :reload-all))

(let [state (ref true)]
  
  (defn start []
    (let [capture (capture-from-cam 0)
          cascade (load-cascade "/opt/local/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")]
      (dosync (ref-set state true))
      (future
       (while @state
         (let [curr (query-frame capture)
               faces (haar-detect-objects curr cascade 1.1 2 :haar-do-canny-prunning [40 40])]

           (doseq [[x y w h] faces] 
             (rectangle curr x y w h java.awt.Color/red 5))
           
           (show-image :cam curr)))
       (release-capture capture))))

  (defn stop []
    (dosync (ref-set state false))))
