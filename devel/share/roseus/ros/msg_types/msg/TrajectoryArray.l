;; Auto-generated. Do not edit!


(when (boundp 'msg_types::TrajectoryArray)
  (if (not (find-package "MSG_TYPES"))
    (make-package "MSG_TYPES"))
  (shadow 'TrajectoryArray (find-package "MSG_TYPES")))
(unless (find-package "MSG_TYPES::TRAJECTORYARRAY")
  (make-package "MSG_TYPES::TRAJECTORYARRAY"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryArray.msg.html


(defclass msg_types::TrajectoryArray
  :super ros::object
  :slots (_stamp _trajectories ))

(defmethod msg_types::TrajectoryArray
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:trajectories __trajectories) ())
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _trajectories __trajectories)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:trajectories
   (&rest __trajectories)
   (if (keywordp (car __trajectories))
       (send* _trajectories __trajectories)
     (progn
       (if __trajectories (setq _trajectories (car __trajectories)))
       _trajectories)))
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; msg_types/PositionArray[] _trajectories
    (apply #'+ (send-all _trajectories :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; msg_types/PositionArray[] _trajectories
     (write-long (length _trajectories) s)
     (dolist (elem _trajectories)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; msg_types/PositionArray[] _trajectories
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectories (let (r) (dotimes (i n) (push (instance msg_types::PositionArray :init) r)) r))
     (dolist (elem- _trajectories)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get msg_types::TrajectoryArray :md5sum-) "7766634badb769863e045242311bbf96")
(setf (get msg_types::TrajectoryArray :datatype-) "msg_types/TrajectoryArray")
(setf (get msg_types::TrajectoryArray :definition-)
      "time stamp
msg_types/PositionArray[] trajectories


================================================================================
MSG: msg_types/PositionArray
time stamp
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



(provide :msg_types/TrajectoryArray "7766634badb769863e045242311bbf96")


