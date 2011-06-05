(defproject vision "1.0.0-SNAPSHOT"
  :description "OpenCV wrapper for Clojure."
  :dependencies [[org.clojure/clojure "1.2.0"]
                 [org.clojure/clojure-contrib "1.2.0"]
                 [org.clojars.nakkaya/jna "3.2.7"]]
  :jvm-opts ["-Djna.library.path=resources/lib/"]
  :dev-dependencies [[lein-clojars "0.6.0"]])
