;; Auto-generated. Do not edit!


(when (boundp 'comp0037_mapper::RequestMapUpdate)
  (if (not (find-package "COMP0037_MAPPER"))
    (make-package "COMP0037_MAPPER"))
  (shadow 'RequestMapUpdate (find-package "COMP0037_MAPPER")))
(unless (find-package "COMP0037_MAPPER::REQUESTMAPUPDATE")
  (make-package "COMP0037_MAPPER::REQUESTMAPUPDATE"))
(unless (find-package "COMP0037_MAPPER::REQUESTMAPUPDATEREQUEST")
  (make-package "COMP0037_MAPPER::REQUESTMAPUPDATEREQUEST"))
(unless (find-package "COMP0037_MAPPER::REQUESTMAPUPDATERESPONSE")
  (make-package "COMP0037_MAPPER::REQUESTMAPUPDATERESPONSE"))

(in-package "ROS")





(defclass comp0037_mapper::RequestMapUpdateRequest
  :super ros::object
  :slots (_deltaOccupancyGridRequired ))

(defmethod comp0037_mapper::RequestMapUpdateRequest
  (:init
   (&key
    ((:deltaOccupancyGridRequired __deltaOccupancyGridRequired) nil)
    )
   (send-super :init)
   (setq _deltaOccupancyGridRequired __deltaOccupancyGridRequired)
   self)
  (:deltaOccupancyGridRequired
   (&optional __deltaOccupancyGridRequired)
   (if __deltaOccupancyGridRequired (setq _deltaOccupancyGridRequired __deltaOccupancyGridRequired)) _deltaOccupancyGridRequired)
  (:serialization-length
   ()
   (+
    ;; bool _deltaOccupancyGridRequired
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _deltaOccupancyGridRequired
       (if _deltaOccupancyGridRequired (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _deltaOccupancyGridRequired
     (setq _deltaOccupancyGridRequired (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass comp0037_mapper::RequestMapUpdateResponse
  :super ros::object
  :slots (_initialMapUpdate ))

(defmethod comp0037_mapper::RequestMapUpdateResponse
  (:init
   (&key
    ((:initialMapUpdate __initialMapUpdate) (instance comp0037_mapper::MapUpdate :init))
    )
   (send-super :init)
   (setq _initialMapUpdate __initialMapUpdate)
   self)
  (:initialMapUpdate
   (&rest __initialMapUpdate)
   (if (keywordp (car __initialMapUpdate))
       (send* _initialMapUpdate __initialMapUpdate)
     (progn
       (if __initialMapUpdate (setq _initialMapUpdate (car __initialMapUpdate)))
       _initialMapUpdate)))
  (:serialization-length
   ()
   (+
    ;; comp0037_mapper/MapUpdate _initialMapUpdate
    (send _initialMapUpdate :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; comp0037_mapper/MapUpdate _initialMapUpdate
       (send _initialMapUpdate :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; comp0037_mapper/MapUpdate _initialMapUpdate
     (send _initialMapUpdate :deserialize buf ptr-) (incf ptr- (send _initialMapUpdate :serialization-length))
   ;;
   self)
  )

(defclass comp0037_mapper::RequestMapUpdate
  :super ros::object
  :slots ())

(setf (get comp0037_mapper::RequestMapUpdate :md5sum-) "e5aa3588d7db3769375365d7ca3672e4")
(setf (get comp0037_mapper::RequestMapUpdate :datatype-) "comp0037_mapper/RequestMapUpdate")
(setf (get comp0037_mapper::RequestMapUpdate :request) comp0037_mapper::RequestMapUpdateRequest)
(setf (get comp0037_mapper::RequestMapUpdate :response) comp0037_mapper::RequestMapUpdateResponse)

(defmethod comp0037_mapper::RequestMapUpdateRequest
  (:response () (instance comp0037_mapper::RequestMapUpdateResponse :init)))

(setf (get comp0037_mapper::RequestMapUpdateRequest :md5sum-) "e5aa3588d7db3769375365d7ca3672e4")
(setf (get comp0037_mapper::RequestMapUpdateRequest :datatype-) "comp0037_mapper/RequestMapUpdateRequest")
(setf (get comp0037_mapper::RequestMapUpdateRequest :definition-)
      "bool deltaOccupancyGridRequired
---
comp0037_mapper/MapUpdate initialMapUpdate

================================================================================
MSG: comp0037_mapper/MapUpdate
Header header

bool isPriorMap

float32 scale
int16[] extentInCells
float32 resolution

float32[] occupancyGrid
float32[] deltaOccupancyGrid
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

(setf (get comp0037_mapper::RequestMapUpdateResponse :md5sum-) "e5aa3588d7db3769375365d7ca3672e4")
(setf (get comp0037_mapper::RequestMapUpdateResponse :datatype-) "comp0037_mapper/RequestMapUpdateResponse")
(setf (get comp0037_mapper::RequestMapUpdateResponse :definition-)
      "bool deltaOccupancyGridRequired
---
comp0037_mapper/MapUpdate initialMapUpdate

================================================================================
MSG: comp0037_mapper/MapUpdate
Header header

bool isPriorMap

float32 scale
int16[] extentInCells
float32 resolution

float32[] occupancyGrid
float32[] deltaOccupancyGrid
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



(provide :comp0037_mapper/RequestMapUpdate "e5aa3588d7db3769375365d7ca3672e4")


