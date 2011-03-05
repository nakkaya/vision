(ns camshift
  (:use [vision core] :reload-all))

(defn listener [state]
  (proxy [java.awt.event.MouseListener] []
    (mouseClicked [e])
    (mousePressed [e]
                  (dosync (alter state assoc :upper-left [(.getX e) (.getY e)] :tracker nil)))
    (mouseReleased [e]
                   (let [{:keys [upper-left frame]} @state
                         [x1 y1] upper-left
                         x2 (.getX e) y2 (.getY e) ;;lower right
                         box [x1 y1 (- x2 x1) (- y2 y1)]] 
                     (dosync (alter state assoc :tracker (camshift-init frame box 10 256 30))
                             (alter state dissoc :upper-left))))
    (mouseEntered [e])
    (mouseExited [e])))

(let [state (ref {:loop? true :tracker nil :p1 nil :p2 nil})]
  
  (defn start []
    (let [capture (capture-from-cam 0)]
      
      (dosync (alter state assoc :loop? true))
      (view :cam (query-frame capture))
      (.addMouseListener (-> @*frames* :cam :panel) (listener state))
      
      (future
       (while (:loop? @state)
         (dosync (alter state assoc :frame (clone-image (query-frame capture))))
         
         (let [{:keys [frame tracker]} @state]
           (if-not (nil? tracker)
             (ellipse-box frame (camshift-track frame tracker) java.awt.Color/red 2))
         
           (view :cam frame)
           (release frame)))
       (release capture))))

  (defn stop []
    (dosync (alter state assoc :loop? false))))
