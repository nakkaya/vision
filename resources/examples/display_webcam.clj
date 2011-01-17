(ns display-webcam
  (:use [vision core graphics] :reload-all))

(let [state (ref true)]
  
  (defn start []
    (let [capture (capture-from-cam 0)]
      (dosync (ref-set state true))
      (future
       (while @state
         (show-image :cam (query-frame capture)))
       (release-capture capture))))

  (defn stop []
    (dosync (ref-set state false))))
