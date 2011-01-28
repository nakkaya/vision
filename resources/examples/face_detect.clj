(ns face-detect
  (:use [vision core] :reload-all))

(let [state (ref true)]
  
  (defn start []
    (let [capture (capture-from-cam 0)
          cascade (load-cascade "/opt/local/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")]
      (dosync (ref-set state true))
      (future
       (while @state
         (let [curr (query-frame capture)
               faces (haar-detect-objects curr cascade 1.1 2 :haar-do-canny-prunning [40 40] [100 100])]

           (doseq [[x y w h] faces] 
             (rectangle curr [x y] [(+ w x) (+ h y)] java.awt.Color/red 5))
           
           (view :cam curr)))
       (release capture))))

  (defn stop []
    (dosync (ref-set state false))))
