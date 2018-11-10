; Auto-generated. Do not edit!


(cl:in-package mastering_ros_demo_pkg-msg)


;//! \htmlinclude demo_msg.msg.html

(cl:defclass <demo_msg> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:float
    :initform 0.0))
)

(cl:defclass demo_msg (<demo_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <demo_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'demo_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mastering_ros_demo_pkg-msg:<demo_msg> is deprecated: use mastering_ros_demo_pkg-msg:demo_msg instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <demo_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mastering_ros_demo_pkg-msg:number-val is deprecated.  Use mastering_ros_demo_pkg-msg:number instead.")
  (number m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <demo_msg>) ostream)
  "Serializes a message object of type '<demo_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <demo_msg>) istream)
  "Deserializes a message object of type '<demo_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'number) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<demo_msg>)))
  "Returns string type for a message object of type '<demo_msg>"
  "mastering_ros_demo_pkg/demo_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'demo_msg)))
  "Returns string type for a message object of type 'demo_msg"
  "mastering_ros_demo_pkg/demo_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<demo_msg>)))
  "Returns md5sum for a message object of type '<demo_msg>"
  "ded049c24c756963282afab14b2d0f6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'demo_msg)))
  "Returns md5sum for a message object of type 'demo_msg"
  "ded049c24c756963282afab14b2d0f6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<demo_msg>)))
  "Returns full string definition for message of type '<demo_msg>"
  (cl:format cl:nil "float32 number~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'demo_msg)))
  "Returns full string definition for message of type 'demo_msg"
  (cl:format cl:nil "float32 number~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <demo_msg>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <demo_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'demo_msg
    (cl:cons ':number (number msg))
))
