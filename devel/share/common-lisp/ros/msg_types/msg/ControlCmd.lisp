; Auto-generated. Do not edit!


(cl:in-package msg_types-msg)


;//! \htmlinclude ControlCmd.msg.html

(cl:defclass <ControlCmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlCmd (<ControlCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msg_types-msg:<ControlCmd> is deprecated: use msg_types-msg:ControlCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:header-val is deprecated.  Use msg_types-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <ControlCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:v-val is deprecated.  Use msg_types-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <ControlCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:w-val is deprecated.  Use msg_types-msg:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlCmd>) ostream)
  "Serializes a message object of type '<ControlCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlCmd>) istream)
  "Deserializes a message object of type '<ControlCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlCmd>)))
  "Returns string type for a message object of type '<ControlCmd>"
  "msg_types/ControlCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlCmd)))
  "Returns string type for a message object of type 'ControlCmd"
  "msg_types/ControlCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlCmd>)))
  "Returns md5sum for a message object of type '<ControlCmd>"
  "c9739f01512ce85d8ac1ccdd6bde650b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlCmd)))
  "Returns md5sum for a message object of type 'ControlCmd"
  "c9739f01512ce85d8ac1ccdd6bde650b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlCmd>)))
  "Returns full string definition for message of type '<ControlCmd>"
  (cl:format cl:nil "Header header~%float32 v~%float32 w~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlCmd)))
  "Returns full string definition for message of type 'ControlCmd"
  (cl:format cl:nil "Header header~%float32 v~%float32 w~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlCmd
    (cl:cons ':header (header msg))
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
))
