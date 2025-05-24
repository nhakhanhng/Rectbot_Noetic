;; Auto-generated. Do not edit!


(when (boundp 'rectbot_cv::KeyPoints)
  (if (not (find-package "RECTBOT_CV"))
    (make-package "RECTBOT_CV"))
  (shadow 'KeyPoints (find-package "RECTBOT_CV")))
(unless (find-package "RECTBOT_CV::KEYPOINTS")
  (make-package "RECTBOT_CV::KEYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude KeyPoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rectbot_cv::KeyPoints
  :super ros::object
  :slots (_point _score ))

(defmethod rectbot_cv::KeyPoints
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::Point32 :init))
    ((:score __score) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _point __point)
   (setq _score __score)
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:score
   (&rest __score)
   (if (keywordp (car __score))
       (send* _score __score)
     (progn
       (if __score (setq _score (car __score)))
       _score)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point32 _point
    (send _point :serialization-length)
    ;; std_msgs/Float32 _score
    (send _score :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point32 _point
       (send _point :serialize s)
     ;; std_msgs/Float32 _score
       (send _score :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point32 _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; std_msgs/Float32 _score
     (send _score :deserialize buf ptr-) (incf ptr- (send _score :serialization-length))
   ;;
   self)
  )

(setf (get rectbot_cv::KeyPoints :md5sum-) "92f94618554fe5449a29ca0200149d32")
(setf (get rectbot_cv::KeyPoints :datatype-) "rectbot_cv/KeyPoints")
(setf (get rectbot_cv::KeyPoints :definition-)
      "geometry_msgs/Point32 point
std_msgs/Float32 score
================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :rectbot_cv/KeyPoints "92f94618554fe5449a29ca0200149d32")


