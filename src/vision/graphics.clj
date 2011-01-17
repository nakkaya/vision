(ns vision.graphics
  (:use [vision core util]))

(defn- image-panel [image]
  (proxy [javax.swing.JPanel] []
    (paintComponent [g] (.drawImage g image 0 0 this))))

(defn color-picker
  "Displays the image in a frame with a mouse listener attached that will
   print the HSV values for the pixels clicked."
  [[pointer image]]
  (let [listener (proxy [java.awt.event.MouseListener] []
                   (mouseClicked
                    [e]
                    (let [x (.getX e) y (.getY e)
                          c (java.awt.Color.
                             (.getRGB @image  x y))
                          hsb (java.awt.Color/RGBtoHSB
                               (.getRed c) (.getGreen c) (.getBlue c) nil)]
                      (println x y (map #(map-int % 0 1 0 179) (seq hsb)))))
                   (mousePressed [e])
                   (mouseReleased [e])
                   (mouseEntered [e])
                   (mouseExited [e]))
        panel  (doto (image-panel @image)
                 (.addMouseListener listener))]
    (doto (javax.swing.JFrame.)
      (.add panel)
      (.setAlwaysOnTop true)
      (.setSize (java.awt.Dimension. (.getWidth @image) (.getHeight @image)))
      (.setVisible true))))

(defn circle
  "Draws simple, thick or filled circle."
  [[pointer image] [x y r] color thickness]
  (let [g (.getGraphics @image)]
    (.setColor g color)
  (if (pos? thickness)
    (doto g
      (.setStroke (java.awt.BasicStroke. thickness))
      (.draw (java.awt.geom.Ellipse2D$Double. (- x r) (- y r) (* 2 r) (* 2 r))))
    (.fill g (java.awt.geom.Ellipse2D$Double. (- x r) (- y r) (* 2 r) (* 2 r))))))

(defn line
  "Draws simple or thick line segment."
  [[pointer image] [x1 y1] [x2 y2] color thickness]
  (doto (.getGraphics @image)
    (.setColor color)
    (.setStroke (java.awt.BasicStroke. thickness))
    (.drawLine x1 y1 x2 y2)))

(defn rectangle
  "Draws simple or thick rectangle."
  [[pointer image] x y width height color thickness]
  (let [g (.getGraphics @image)]
    (.setColor g color)
  (if (pos? thickness)
    (doto g
      (.setStroke (java.awt.BasicStroke. thickness))
      (.drawRect x y width height))
    (.fillRect x y width height))))

(def *frames* (ref {}))

(defn- image-panel [f]
  (let [p (proxy [javax.swing.JPanel] []
            (paintComponent [g] (.drawImage g (-> @*frames* f :image) 0 0 this))
            (getPreferredSize [] (java.awt.Dimension.
                                  (.getWidth (-> @*frames* f :image))
                                  (.getHeight (-> @*frames* f :image)))))]
    (add-watch *frames* (str f) (fn [k r o n] (.repaint p)))
    p))

(defn- window-adapter [f]
  (proxy [java.awt.event.WindowAdapter] [] 
    (windowClosing [e] (dosync (alter *frames* dissoc f)))))

(defn- image-frame [f]
  (let [frame (doto (javax.swing.JFrame. (str f))
                (.add (image-panel f))
                (.setAlwaysOnTop true)
                (.pack)
                (.addWindowListener (window-adapter f))
                (.setVisible true))]
    (dosync (alter *frames* assoc-in [f :frame] frame))))

(defn show-image
  "Displays the image in a frame."
  [f [_ i]]
  (dosync (alter *frames* assoc-in [f :image] @i))  
  (when (nil? (-> @*frames* f :frame))
    (image-frame f)))
