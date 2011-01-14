(ns vision.util)

(defn map-int [x in-min in-max out-min out-max]
  (+ (/ (* (- x in-min) (- out-max out-min)) (- in-max in-min)) out-min))

(defn in-range? [x [a b]]
  (if (and (>= x a)
           (<= x b))
    true false))
