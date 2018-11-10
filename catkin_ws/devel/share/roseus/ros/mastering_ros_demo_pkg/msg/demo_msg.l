;; Auto-generated. Do not edit!


(when (boundp 'mastering_ros_demo_pkg::demo_msg)
  (if (not (find-package "MASTERING_ROS_DEMO_PKG"))
    (make-package "MASTERING_ROS_DEMO_PKG"))
  (shadow 'demo_msg (find-package "MASTERING_ROS_DEMO_PKG")))
(unless (find-package "MASTERING_ROS_DEMO_PKG::DEMO_MSG")
  (make-package "MASTERING_ROS_DEMO_PKG::DEMO_MSG"))

(in-package "ROS")
;;//! \htmlinclude demo_msg.msg.html


(defclass mastering_ros_demo_pkg::demo_msg
  :super ros::object
  :slots (_number ))

(defmethod mastering_ros_demo_pkg::demo_msg
  (:init
   (&key
    ((:number __number) 0.0)
    )
   (send-super :init)
   (setq _number (float __number))
   self)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:serialization-length
   ()
   (+
    ;; float32 _number
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _number
       (sys::poke _number (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _number
     (setq _number (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mastering_ros_demo_pkg::demo_msg :md5sum-) "ded049c24c756963282afab14b2d0f6d")
(setf (get mastering_ros_demo_pkg::demo_msg :datatype-) "mastering_ros_demo_pkg/demo_msg")
(setf (get mastering_ros_demo_pkg::demo_msg :definition-)
      "float32 number
")



(provide :mastering_ros_demo_pkg/demo_msg "ded049c24c756963282afab14b2d0f6d")


