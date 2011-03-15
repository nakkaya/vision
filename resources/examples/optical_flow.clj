(ns optical-flow
  (:use [vision core] :reload-all))

(let [state (ref {:loop true})]
  
  (defn start []
    (let [capture (capture-from-cam 0)]
      
      (dosync (alter state assoc
                     :loop true
                     :prev (clone-image (query-frame capture))
                     :features (good-features-to-track (query-frame capture) 500 0.01 10 10)))
      (future
       (while (:loop @state)
         (let [{:keys [prev features]} @state
               frame (query-frame capture)
               features (calc-optical-flow-pyr-lk prev frame features 10 3)]
           
           (doseq [[x y] features]
             (circle frame [(int x) (int y)] 3 java.awt.Color/green -1))
           
           (view :cam frame)
           
           (dosync (alter state assoc
                          :prev (clone-image frame)
                          :features features))
           
           (release prev)))
       (release capture))))

  (defn stop []
    (dosync (alter state assoc :loop false))))
