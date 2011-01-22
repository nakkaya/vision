(ns robocup-id
  (:refer-clojure :exclude [+ - * =])
  (:use (clojure.contrib.generic [arithmetic :only [+ - *]]
                                 [comparison :only [=]]))
  (:use [vector-2d.core] :reload-all)
  (:use [vision core] :reload-all))

(defn in-range? [x [a b]]
  (if (and (>= x a)
           (<= x b))
    true false))

(defn id-10 [team pink]
  (let [pink (map #(apply vector-2d (take 2 %)) pink)]
    (reduce (fn[h v]
              (let [self (apply vector-2d (take 2 v))
                    [f s] (take 2 (sort-by #(dist self %) pink))]
                (if (and (in-range? (dist self f) [50 60])
                         (in-range? (dist self s) [50 60])
                         (= -2 (int (bearing self f)))
                         (= 2 (int (bearing self s))))
                  (take 2 v) h))) nil team)))

(defn id-09 [team pink]
  (let [pink (map #(apply vector-2d (take 2 %)) pink)]
    (reduce (fn[h v]
              (let [self (apply vector-2d (take 2 v))
                    [f s] (take 2 (sort-by #(dist self %) pink))]
                (if (and (in-range? (dist self f) [50 60])
                         (in-range? (dist self s) [50 60])
                         (= -1 (int (bearing self f)))
                         (= 1 (int (bearing self s))))
                  (take 2 v) h))) nil team)))

(let [image (load-image "resources/samples/robocup-marker.png" :color)
      blue  (--> (convert-color image :bgr-hsv)
                 (in-range-s [115 0 0 0] [125 255 255 0])
                 (smooth :gaussian 9 9 0 0)
                 (hough-circles 2 45 100 40 10 40))
      
      pink  (--> (convert-color image :bgr-hsv)
                 (in-range-s [145 0 0 0] [165 255 255 0])
                 (smooth :gaussian 9 9 0 0)
                 (hough-circles 2 35 100 40 10 30))]

  (doseq [c pink]
    (circle image (take 2 c) 10 java.awt.Color/yellow 3))
  
  (circle image (id-10 blue pink) 20 java.awt.Color/red 10)
  (circle image (id-09 blue pink) 20 java.awt.Color/red 10)
  
  (view :result image))
