  (ns display-webcam
    (:use [vision core graphics])
    (:import (javax.swing JFrame)
             (java.awt.event ActionListener KeyListener)))
  
  (defn display []
    (let [capture (capture-from-cam 0)
          state (ref {:loop true :img (query-frame capture)})
          frame (proxy [JFrame KeyListener] [] 
                  (paint [g] (.drawImage g @(second (:img @state)) 0 0 nil))
                  (keyPressed [e])
                  (keyReleased [e])
                  (keyTyped [e] (dosync (alter state assoc :loop false))))]
      
      (future
       (while (:loop @state)
         (dosync (alter state assoc :img (query-frame capture)))
         (.repaint frame))
       (release-capture capture)
       (.setVisible frame false))
      
      (doto frame
        (.addKeyListener frame)
        (.setSize 640 480)
        (.show))))

;;(display)
