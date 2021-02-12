;; Auto-generated. Do not edit!


(when (boundp 'msg_types::StateArray)
  (if (not (find-package "MSG_TYPES"))
    (make-package "MSG_TYPES"))
  (shadow 'StateArray (find-package "MSG_TYPES")))
(unless (find-package "MSG_TYPES::STATEARRAY")
  (make-package "MSG_TYPES::STATEARRAY"))

(in-package "ROS")
;;//! \htmlinclude StateArray.msg.html


(defclass msg_types::StateArray
  :super ros::object
  :slots (_stamp _desired_path _sync_predictions _initial_state ))

(defmethod msg_types::StateArray
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:desired_path __desired_path) ())
    ((:sync_predictions __sync_predictions) (instance msg_types::TrajectoryArray :init))
    ((:initial_state __initial_state) (instance msg_types::State :init))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _desired_path __desired_path)
   (setq _sync_predictions __sync_predictions)
   (setq _initial_state __initial_state)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:desired_path
   (&rest __desired_path)
   (if (keywordp (car __desired_path))
       (send* _desired_path __desired_path)
     (progn
       (if __desired_path (setq _desired_path (car __desired_path)))
       _desired_path)))
  (:sync_predictions
   (&rest __sync_predictions)
   (if (keywordp (car __sync_predictions))
       (send* _sync_predictions __sync_predictions)
     (progn
       (if __sync_predictions (setq _sync_predictions (car __sync_predictions)))
       _sync_predictions)))
  (:initial_state
   (&rest __initial_state)
   (if (keywordp (car __initial_state))
       (send* _initial_state __initial_state)
     (progn
       (if __initial_state (setq _initial_state (car __initial_state)))
       _initial_state)))
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; msg_types/State[] _desired_path
    (apply #'+ (send-all _desired_path :serialization-length)) 4
    ;; msg_types/TrajectoryArray _sync_predictions
    (send _sync_predictions :serialization-length)
    ;; msg_types/State _initial_state
    (send _initial_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; msg_types/State[] _desired_path
     (write-long (length _desired_path) s)
     (dolist (elem _desired_path)
       (send elem :serialize s)
       )
     ;; msg_types/TrajectoryArray _sync_predictions
       (send _sync_predictions :serialize s)
     ;; msg_types/State _initial_state
       (send _initial_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; msg_types/State[] _desired_path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _desired_path (let (r) (dotimes (i n) (push (instance msg_types::State :init) r)) r))
     (dolist (elem- _desired_path)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; msg_types/TrajectoryArray _sync_predictions
     (send _sync_predictions :deserialize buf ptr-) (incf ptr- (send _sync_predictions :serialization-length))
   ;; msg_types/State _initial_state
     (send _initial_state :deserialize buf ptr-) (incf ptr- (send _initial_state :serialization-length))
   ;;
   self)
  )

(setf (get msg_types::StateArray :md5sum-) "f6495b47ab8804317c3ad5428e9da0a5")
(setf (get msg_types::StateArray :datatype-) "msg_types/StateArray")
(setf (get msg_types::StateArray :definition-)
      "time stamp
msg_types/State[] desired_path
msg_types/TrajectoryArray sync_predictions
msg_types/State initial_state


================================================================================
MSG: msg_types/State
time stamp
float32 x
float32 y
float32 heading
float32 v
float32 w


================================================================================
MSG: msg_types/TrajectoryArray
time stamp
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



(provide :msg_types/StateArray "f6495b47ab8804317c3ad5428e9da0a5")


