;; Auto-generated. Do not edit!


(when (boundp 'msg_types::PositionArray)
  (if (not (find-package "MSG_TYPES"))
    (make-package "MSG_TYPES"))
  (shadow 'PositionArray (find-package "MSG_TYPES")))
(unless (find-package "MSG_TYPES::POSITIONARRAY")
  (make-package "MSG_TYPES::POSITIONARRAY"))

(in-package "ROS")
;;//! \htmlinclude PositionArray.msg.html


(defclass msg_types::PositionArray
  :super ros::object
  :slots (_stamp _objects ))

(defmethod msg_types::PositionArray
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:objects __objects) (let (r) (dotimes (i 0) (push (instance msg_types::Position :init) r)) r))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _objects __objects)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; msg_types/Position[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; msg_types/Position[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; msg_types/Position[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance msg_types::Position :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get msg_types::PositionArray :md5sum-) "55fbca85e311ffe813e31fb2e1229ad7")
(setf (get msg_types::PositionArray :datatype-) "msg_types/PositionArray")
(setf (get msg_types::PositionArray :definition-)
      "time stamp
msg_types/Position[] objects


================================================================================
MSG: msg_types/Position
time stamp
float32 x
float32 y
float32 t
uint32 id
bool actual

")



(provide :msg_types/PositionArray "55fbca85e311ffe813e31fb2e1229ad7")


