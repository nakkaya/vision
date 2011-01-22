(ns shape-match
  (:use [vision core]))

(let [raw (load-image "resources/samples/shape1.png" :grayscale)
      match (load-image "resources/samples/shape1b_I1.png" :grayscale)
      no-match (load-image "resources/samples/shape1rot_I1.png" :grayscale)]
  (println (match-shapes raw match :i1))
  (println (match-shapes raw no-match :i1)))
