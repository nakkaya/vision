(ns listener
  (:use [vision core] :reload-all))

(defn map-int [x in-min in-max out-min out-max]
  (+ (/ (* (- x in-min) (- out-max out-min)) (- in-max in-min)) out-min))

(defn listener [image]
  (proxy [java.awt.event.MouseListener] []
    (mouseClicked
     [e]
     (let [x (.getX e) y (.getY e)
           c (java.awt.Color.
              (.getRGB image  x y))
           hsb (java.awt.Color/RGBtoHSB
                (.getRed c) (.getGreen c) (.getBlue c) nil)]
       (println x y (map #(map-int % 0 1 0 179) (seq hsb)))))
    (mousePressed [e])
    (mouseReleased [e])
    (mouseEntered [e])
    (mouseExited [e])))

(let [img (load-image "resources/samples/robocup-marker.png" :color)]
  (view :img img)
  (.addMouseListener (-> @*frames* :img :panel) (listener (-> @*frames* :img :image)))
  (release img))
