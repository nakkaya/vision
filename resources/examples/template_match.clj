(ns template-match
  (:use [vision core] :reload-all))

(let [raw (load-image "resources/samples/template-input.png" :color)
      template (load-image "resources/samples/template-match.png" :color)
      [width height] (image-size template)
      [min max min-x min-y max-x max-y] (match-template raw template :ccorr-normed)]
  (rectangle raw [max-x max-y] [(+ max-x width) (+ max-y height)] java.awt.Color/blue 2)
  (view :result raw))
