;; Auto-generated. Do not edit!


(when (boundp 'rectbot_cv::KeyPoint)
  (if (not (find-package "RECTBOT_CV"))
    (make-package "RECTBOT_CV"))
  (shadow 'KeyPoint (find-package "RECTBOT_CV")))
(unless (find-package "RECTBOT_CV::KEYPOINT")
  (make-package "RECTBOT_CV::KEYPOINT"))

(in-package "ROS")
;;//! \htmlinclude KeyPoint.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass rectbot_cv::KeyPoint
  :super ros::object
  :slots (_x _y _score ))

(defmethod rectbot_cv::KeyPoint
  (:init
   (&key
    ((:x __x) (instance std_msgs::Float32 :init))
    ((:y __y) (instance std_msgs::Float32 :init))
    ((:score __score) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _score __score)
   self)
  (:x
   (&rest __x)
   (if (keywordp (car __x))
       (send* _x __x)
     (progn
       (if __x (setq _x (car __x)))
       _x)))
  (:y
   (&rest __y)
   (if (keywordp (car __y))
       (send* _y __y)
     (progn
       (if __y (setq _y (car __y)))
       _y)))
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
    ;; std_msgs/Float32 _x
    (send _x :serialization-length)
    ;; std_msgs/Float32 _y
    (send _y :serialization-length)
    ;; std_msgs/Float32 _score
    (send _score :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32 _x
       (send _x :serialize s)
     ;; std_msgs/Float32 _y
       (send _y :serialize s)
     ;; std_msgs/Float32 _score
       (send _score :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32 _x
     (send _x :deserialize buf ptr-) (incf ptr- (send _x :serialization-length))
   ;; std_msgs/Float32 _y
     (send _y :deserialize buf ptr-) (incf ptr- (send _y :serialization-length))
   ;; std_msgs/Float32 _score
     (send _score :deserialize buf ptr-) (incf ptr- (send _score :serialization-length))
   ;;
   self)
  )

(setf (get rectbot_cv::KeyPoint :md5sum-) "1cd825e8df87066e1486d6a8d6cfd264")
(setf (get rectbot_cv::KeyPoint :datatype-) "rectbot_cv/KeyPoint")
(setf (get rectbot_cv::KeyPoint :definition-)
      "std_msgs/Float32 x
std_msgs/Float32 y
std_msgs/Float32 score
================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :rectbot_cv/KeyPoint "1cd825e8df87066e1486d6a8d6cfd264")


