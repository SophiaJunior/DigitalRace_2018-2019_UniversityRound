
(cl:in-package :asdf)

(defsystem "mastering_ros_demo_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "demo_srv" :depends-on ("_package_demo_srv"))
    (:file "_package_demo_srv" :depends-on ("_package"))
  ))