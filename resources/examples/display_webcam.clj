(ns display-webcam
  (:use [vision core] :reload-all))

(let [state (ref true)]
  
  (defn start []
    (let [capture (capture-from-cam 0)]
      (dosync (ref-set state true))
      (future
       (while @state
         (view :cam (query-frame capture)))
       (release capture))))

  (defn stop []
    (dosync (ref-set state false))))
