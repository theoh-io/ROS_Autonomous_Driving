;; Auto-generated. Do not edit!


(when (boundp 'msg_types::ControlCmd)
  (if (not (find-package "MSG_TYPES"))
    (make-package "MSG_TYPES"))
  (shadow 'ControlCmd (find-package "MSG_TYPES")))
(unless (find-package "MSG_TYPES::CONTROLCMD")
  (make-package "MSG_TYPES::CONTROLCMD"))

(in-package "ROS")
;;//! \htmlinclude ControlCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msg_types::ControlCmd
  :super ros::object
  :slots (_header _v _w ))

(defmethod msg_types::ControlCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:v __v) 0.0)
    ((:w __w) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _v (float __v))
   (setq _w (float __w))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _v
    4
    ;; float32 _w
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _v
       (sys::poke _v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _w
       (sys::poke _w (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _v
     (setq _v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _w
     (setq _w (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get msg_types::ControlCmd :md5sum-) "c9739f01512ce85d8ac1ccdd6bde650b")
(setf (get msg_types::ControlCmd :datatype-) "msg_types/ControlCmd")
(setf (get msg_types::ControlCmd :definition-)
      "Header header
float32 v
float32 w


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :msg_types/ControlCmd "c9739f01512ce85d8ac1ccdd6bde650b")


