; Auto-generated. Do not edit!


(cl:in-package mastering_ros_demo_pkg-srv)


;//! \htmlinclude demo_srv-request.msg.html

(cl:defclass <demo_srv-request> (roslisp-msg-protocol:ros-message)
  ((in
    :reader in
    :initarg :in
    :type cl:string
    :initform "")
   (out
    :reader out
    :initarg :out
    :type cl:string
    :initform ""))
)

(cl:defclass demo_srv-request (<demo_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <demo_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'demo_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mastering_ros_demo_pkg-srv:<demo_srv-request> is deprecated: use mastering_ros_demo_pkg-srv:demo_srv-request instead.")))

(cl:ensure-generic-function 'in-val :lambda-list '(m))
(cl:defmethod in-val ((m <demo_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mastering_ros_demo_pkg-srv:in-val is deprecated.  Use mastering_ros_demo_pkg-srv:in instead.")
  (in m))

(cl:ensure-generic-function 'out-val :lambda-list '(m))
(cl:defmethod out-val ((m <demo_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mastering_ros_demo_pkg-srv:out-val is deprecated.  Use mastering_ros_demo_pkg-srv:out instead.")
  (out m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <demo_srv-request>) ostream)
  "Serializes a message object of type '<demo_srv-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'in))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'in))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'out))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'out))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <demo_srv-request>) istream)
  "Deserializes a message object of type '<demo_srv-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'in) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'in) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'out) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'out) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<demo_srv-request>)))
  "Returns string type for a service object of type '<demo_srv-request>"
  "mastering_ros_demo_pkg/demo_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'demo_srv-request)))
  "Returns string type for a service object of type 'demo_srv-request"
  "mastering_ros_demo_pkg/demo_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<demo_srv-request>)))
  "Returns md5sum for a message object of type '<demo_srv-request>"
  "6c2950507572d8c02cc39d10ef9010b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'demo_srv-request)))
  "Returns md5sum for a message object of type 'demo_srv-request"
  "6c2950507572d8c02cc39d10ef9010b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<demo_srv-request>)))
  "Returns full string definition for message of type '<demo_srv-request>"
  (cl:format cl:nil "string in~%string out~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'demo_srv-request)))
  "Returns full string definition for message of type 'demo_srv-request"
  (cl:format cl:nil "string in~%string out~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <demo_srv-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'in))
     4 (cl:length (cl:slot-value msg 'out))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <demo_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'demo_srv-request
    (cl:cons ':in (in msg))
    (cl:cons ':out (out msg))
))
;//! \htmlinclude demo_srv-response.msg.html

(cl:defclass <demo_srv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass demo_srv-response (<demo_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <demo_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'demo_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mastering_ros_demo_pkg-srv:<demo_srv-response> is deprecated: use mastering_ros_demo_pkg-srv:demo_srv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <demo_srv-response>) ostream)
  "Serializes a message object of type '<demo_srv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <demo_srv-response>) istream)
  "Deserializes a message object of type '<demo_srv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<demo_srv-response>)))
  "Returns string type for a service object of type '<demo_srv-response>"
  "mastering_ros_demo_pkg/demo_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'demo_srv-response)))
  "Returns string type for a service object of type 'demo_srv-response"
  "mastering_ros_demo_pkg/demo_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<demo_srv-response>)))
  "Returns md5sum for a message object of type '<demo_srv-response>"
  "6c2950507572d8c02cc39d10ef9010b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'demo_srv-response)))
  "Returns md5sum for a message object of type 'demo_srv-response"
  "6c2950507572d8c02cc39d10ef9010b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<demo_srv-response>)))
  "Returns full string definition for message of type '<demo_srv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'demo_srv-response)))
  "Returns full string definition for message of type 'demo_srv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <demo_srv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <demo_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'demo_srv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'demo_srv)))
  'demo_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'demo_srv)))
  'demo_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'demo_srv)))
  "Returns string type for a service object of type '<demo_srv>"
  "mastering_ros_demo_pkg/demo_srv")