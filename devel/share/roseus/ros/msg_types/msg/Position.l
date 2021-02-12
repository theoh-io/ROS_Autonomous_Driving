;; Auto-generated. Do not edit!


(when (boundp 'msg_types::Position)
  (if (not (find-package "MSG_TYPES"))
    (make-package "MSG_TYPES"))
  (shadow 'Position (find-package "MSG_TYPES")))
(unless (find-package "MSG_TYPES::POSITION")
  (make-package "MSG_TYPES::POSITION"))

(in-package "ROS")
;;//! \htmlinclude Position.msg.html


(defclass msg_types::Position
  :super ros::object
  :slots (_stamp _x _y _t _id _actual ))

(defmethod msg_types::Position
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:t __t) 0.0)
    ((:id __id) 0)
    ((:actual __actual) nil)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _t (float __t))
   (setq _id (round __id))
   (setq _actual __actual)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:actual
   (&optional (__actual :null))
   (if (not (eq __actual :null)) (setq _actual __actual)) _actual)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _t
    4
    ;; uint32 _id
    4
    ;; bool _actual
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _t
       (sys::poke _t (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _id
       (write-long _id s)
     ;; bool _actual
       (if _actual (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _t
     (setq _t (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _actual
     (setq _actual (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get msg_types::Position :md5sum-) "ad7ebc1d669ad680b4c5c766755341b3")
(setf (get msg_types::Position :datatype-) "msg_types/Position")
(setf (get msg_types::Position :definition-)
      "time stamp
float32 x
float32 y
float32 t
uint32 id
bool actual

")



(provide :msg_types/Position "ad7ebc1d669ad680b4c5c766755341b3")


