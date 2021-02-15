; Auto-generated. Do not edit!


(cl:in-package msg_types-msg)


;//! \htmlinclude StateArray.msg.html

(cl:defclass <StateArray> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (desired_path
    :reader desired_path
    :initarg :desired_path
    :type (cl:vector msg_types-msg:State)
   :initform (cl:make-array 0 :element-type 'msg_types-msg:State :initial-element (cl:make-instance 'msg_types-msg:State)))
   (sync_predictions
    :reader sync_predictions
    :initarg :sync_predictions
    :type msg_types-msg:TrajectoryArray
    :initform (cl:make-instance 'msg_types-msg:TrajectoryArray))
   (initial_state
    :reader initial_state
    :initarg :initial_state
    :type msg_types-msg:State
    :initform (cl:make-instance 'msg_types-msg:State)))
)

(cl:defclass StateArray (<StateArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msg_types-msg:<StateArray> is deprecated: use msg_types-msg:StateArray instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <StateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:stamp-val is deprecated.  Use msg_types-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'desired_path-val :lambda-list '(m))
(cl:defmethod desired_path-val ((m <StateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:desired_path-val is deprecated.  Use msg_types-msg:desired_path instead.")
  (desired_path m))

(cl:ensure-generic-function 'sync_predictions-val :lambda-list '(m))
(cl:defmethod sync_predictions-val ((m <StateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:sync_predictions-val is deprecated.  Use msg_types-msg:sync_predictions instead.")
  (sync_predictions m))

(cl:ensure-generic-function 'initial_state-val :lambda-list '(m))
(cl:defmethod initial_state-val ((m <StateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_types-msg:initial_state-val is deprecated.  Use msg_types-msg:initial_state instead.")
  (initial_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateArray>) ostream)
  "Serializes a message object of type '<StateArray>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'desired_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'desired_path))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sync_predictions) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'initial_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateArray>) istream)
  "Deserializes a message object of type '<StateArray>"
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
  (cl:setf (cl:slot-value msg 'desired_path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'desired_path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'msg_types-msg:State))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sync_predictions) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'initial_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateArray>)))
  "Returns string type for a message object of type '<StateArray>"
  "msg_types/StateArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateArray)))
  "Returns string type for a message object of type 'StateArray"
  "msg_types/StateArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateArray>)))
  "Returns md5sum for a message object of type '<StateArray>"
  "f6495b47ab8804317c3ad5428e9da0a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateArray)))
  "Returns md5sum for a message object of type 'StateArray"
  "f6495b47ab8804317c3ad5428e9da0a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateArray>)))
  "Returns full string definition for message of type '<StateArray>"
  (cl:format cl:nil "time stamp~%msg_types/State[] desired_path~%msg_types/TrajectoryArray sync_predictions~%msg_types/State initial_state~%~%~%================================================================================~%MSG: msg_types/State~%time stamp~%float32 x~%float32 y~%float32 heading~%float32 v~%float32 w~%~%~%================================================================================~%MSG: msg_types/TrajectoryArray~%time stamp~%msg_types/PositionArray[] trajectories~%~%~%================================================================================~%MSG: msg_types/PositionArray~%time stamp~%msg_types/Position[] objects~%~%~%================================================================================~%MSG: msg_types/Position~%time stamp~%float32 x~%float32 y~%float32 t~%uint32 id~%bool actual~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateArray)))
  "Returns full string definition for message of type 'StateArray"
  (cl:format cl:nil "time stamp~%msg_types/State[] desired_path~%msg_types/TrajectoryArray sync_predictions~%msg_types/State initial_state~%~%~%================================================================================~%MSG: msg_types/State~%time stamp~%float32 x~%float32 y~%float32 heading~%float32 v~%float32 w~%~%~%================================================================================~%MSG: msg_types/TrajectoryArray~%time stamp~%msg_types/PositionArray[] trajectories~%~%~%================================================================================~%MSG: msg_types/PositionArray~%time stamp~%msg_types/Position[] objects~%~%~%================================================================================~%MSG: msg_types/Position~%time stamp~%float32 x~%float32 y~%float32 t~%uint32 id~%bool actual~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateArray>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'desired_path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sync_predictions))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'initial_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateArray>))
  "Converts a ROS message object to a list"
  (cl:list 'StateArray
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':desired_path (desired_path msg))
    (cl:cons ':sync_predictions (sync_predictions msg))
    (cl:cons ':initial_state (initial_state msg))
))
