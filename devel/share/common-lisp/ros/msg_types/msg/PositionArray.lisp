; Auto-generated. Do not edit!


(cl:in-package msg_types-msg)


;//! \htmlinclude PositionArray.msg.html

(cl:defclass <PositionArray> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (objects
    :reader objects
    :initarg :objects
    :type (cl:vector msg_types-msg:Position)
   :initform (cl:make-array 0 :element-type 'msg_types-msg:Position :initial-element (cl:make-instance 'msg_types-msg:Position))))
)

(cl:defclass PositionArray (<PositionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msg_types-msg:<PositionArray> is deprecated: use msg_types-msg:PositionArray instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <PositionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:stamp-val is deprecated.  Use msg_types-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <PositionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:objects-val is deprecated.  Use msg_types-msg:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionArray>) ostream)
  "Serializes a message object of type '<PositionArray>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionArray>) istream)
  "Deserializes a message object of type '<PositionArray>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'msg_types-msg:Position))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionArray>)))
  "Returns string type for a message object of type '<PositionArray>"
  "msg_types/PositionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionArray)))
  "Returns string type for a message object of type 'PositionArray"
  "msg_types/PositionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionArray>)))
  "Returns md5sum for a message object of type '<PositionArray>"
  "55fbca85e311ffe813e31fb2e1229ad7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionArray)))
  "Returns md5sum for a message object of type 'PositionArray"
  "55fbca85e311ffe813e31fb2e1229ad7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionArray>)))
  "Returns full string definition for message of type '<PositionArray>"
  (cl:format cl:nil "time stamp~%msg_types/Position[] objects~%~%~%================================================================================~%MSG: msg_types/Position~%time stamp~%float32 x~%float32 y~%float32 t~%uint32 id~%bool actual~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionArray)))
  "Returns full string definition for message of type 'PositionArray"
  (cl:format cl:nil "time stamp~%msg_types/Position[] objects~%~%~%================================================================================~%MSG: msg_types/Position~%time stamp~%float32 x~%float32 y~%float32 t~%uint32 id~%bool actual~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionArray>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionArray
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':objects (objects msg))
))
