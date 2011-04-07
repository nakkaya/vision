(ns vision.mjpeg
  (:use [vision core])
  (:import (java.net InetAddress ServerSocket Socket SocketException)
           (java.io BufferedReader InputStreamReader)))

(defn- send-ok-response [out]
  (.write out (.getBytes
               (str "HTTP/1.0 200 OK\r\n"
                    "Server: mjpeg-streamer\r\n"
                    "Content-Type: multipart/x-mixed-replace;boundary=informs\r\n"
                    "--informs\r\n"))))

(defn- send-separator [out]
  (.write out (.getBytes (str "\r\n"
                              "--informs\r\n"))))

(defn- send-image [out img quality]
  (let [bytes (encode-image img :jpg quality)
        size (count bytes)]
    (.write out (.getBytes (str "Content-Type: image/jpeg\r\n"
                                "Content-Length: " size "\r\n\r\n")))
    (.write out bytes)))

(defn- mjpeg-server-aux [jpeg-quality #^ServerSocket ss]
  (let [conns (ref [])]
    (future (while (not (.isClosed ss))
              (try
                (println "before...")
                (let [socket (.accept ss)
                      outs (.getOutputStream socket)
                      ins (BufferedReader.
                           (InputStreamReader.
                            (.getInputStream socket)))]
                  (when (= "/" (second (.split (.readLine ins) " ")))
                    (send-ok-response outs)
                    (dosync (alter conns conj socket))))
                (catch Exception e))))
    {:jpeg-quality jpeg-quality :server-socket ss :connections conns}))

(defn mjpeg-server 
  ([jpeg-quality port backlog #^InetAddress bind-addr] 
     (mjpeg-server-aux jpeg-quality (ServerSocket. port backlog bind-addr)))
  ([jpeg-quality port backlog]
     (mjpeg-server-aux jpeg-quality (ServerSocket. port backlog)))
  ([jpeg-quality port]
     (mjpeg-server-aux jpeg-quality (ServerSocket. port))))

(defn mjpeg-stream [server img]
  (let [{:keys [jpeg-quality connections]} server]
    (dosync (ref-set connections
                     (reduce (fn [h v]
                               (try
                                 (doto (.getOutputStream v)
                                   (send-separator)
                                   (send-image img jpeg-quality))
                                 (conj h v)
                                 (catch Exception e h))) [] @connections)))))
