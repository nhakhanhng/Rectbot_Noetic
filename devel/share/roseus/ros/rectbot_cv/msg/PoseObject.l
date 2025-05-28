;; Auto-generated. Do not edit!


(when (boundp 'rectbot_cv::PoseObject)
  (if (not (find-package "RECTBOT_CV"))
    (make-package "RECTBOT_CV"))
  (shadow 'PoseObject (find-package "RECTBOT_CV")))
(unless (find-package "RECTBOT_CV::POSEOBJECT")
  (make-package "RECTBOT_CV::POSEOBJECT"))

(in-package "ROS")
;;//! \htmlinclude PoseObject.msg.html
(if (not (find-package "VISION_MSGS"))
  (ros::roseus-add-msgs "vision_msgs"))


(defclass rectbot_cv::PoseObject
  :super ros::object
  :slots (_detection _keypoints ))

(defmethod rectbot_cv::PoseObject
  (:init
   (&key
    ((:detection __detection) (instance vision_msgs::Detection2D :init))
    ((:keypoints __keypoints) (let (r) (dotimes (i 0) (push (instance rectbot_cv::KeyPoint :init) r)) r))
    )
   (send-super :init)
   (setq _detection __detection)
   (setq _keypoints __keypoints)
   self)
  (:detection
   (&rest __detection)
   (if (keywordp (car __detection))
       (send* _detection __detection)
     (progn
       (if __detection (setq _detection (car __detection)))
       _detection)))
  (:keypoints
   (&rest __keypoints)
   (if (keywordp (car __keypoints))
       (send* _keypoints __keypoints)
     (progn
       (if __keypoints (setq _keypoints (car __keypoints)))
       _keypoints)))
  (:serialization-length
   ()
   (+
    ;; vision_msgs/Detection2D _detection
    (send _detection :serialization-length)
    ;; rectbot_cv/KeyPoint[] _keypoints
    (apply #'+ (send-all _keypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; vision_msgs/Detection2D _detection
       (send _detection :serialize s)
     ;; rectbot_cv/KeyPoint[] _keypoints
     (write-long (length _keypoints) s)
     (dolist (elem _keypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; vision_msgs/Detection2D _detection
     (send _detection :deserialize buf ptr-) (incf ptr- (send _detection :serialization-length))
   ;; rectbot_cv/KeyPoint[] _keypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keypoints (let (r) (dotimes (i n) (push (instance rectbot_cv::KeyPoint :init) r)) r))
     (dolist (elem- _keypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get rectbot_cv::PoseObject :md5sum-) "f36e3669b0713547250eac0a5f98a3b6")
(setf (get rectbot_cv::PoseObject :datatype-) "rectbot_cv/PoseObject")
(setf (get rectbot_cv::PoseObject :definition-)
      "vision_msgs/Detection2D detection
rectbot_cv/KeyPoint[] keypoints
================================================================================
MSG: vision_msgs/Detection2D
# Defines a 2D detection result.
#
# This is similar to a 2D classification, but includes position information,
#   allowing a classification result for a specific crop or image point to
#   to be located in the larger image.

Header header

# Class probabilities
ObjectHypothesisWithPose[] results

# 2D bounding box surrounding the object.
BoundingBox2D bbox

# The 2D data that generated these results (i.e. region proposal cropped out of
#   the image). Not required for all use cases, so it may be empty.
sensor_msgs/Image source_img

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
string frame_id

================================================================================
MSG: vision_msgs/ObjectHypothesisWithPose
# An object hypothesis that contains position information.

# The unique numeric ID of object detected. To get additional information about
#   this ID, such as its human-readable name, listeners should perform a lookup
#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
int64 id

# The probability or confidence value of the detected object. By convention,
#   this value should lie in the range [0-1].
float64 score

# The 6D pose of the object hypothesis. This pose should be
#   defined as the pose of some fixed reference point on the object, such a
#   the geometric center of the bounding box or the center of mass of the
#   object.
# Note that this pose is not stamped; frame information can be defined by
#   parent messages.
# Also note that different classes predicted for the same input data may have
#   different predicted 6D poses.
geometry_msgs/PoseWithCovariance pose
================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: vision_msgs/BoundingBox2D
# A 2D bounding box that can be rotated about its center.
# All dimensions are in pixels, but represented using floating-point
#   values to allow sub-pixel precision. If an exact pixel crop is required
#   for a rotated bounding box, it can be calculated using Bresenham's line
#   algorithm.

# The 2D position (in pixels) and orientation of the bounding box center.
geometry_msgs/Pose2D center

# The size (in pixels) of the bounding box surrounding the object relative
#   to the pose of its center.
float64 size_x
float64 size_y

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: rectbot_cv/KeyPoint
std_msgs/Float32 x
std_msgs/Float32 y
std_msgs/Float32 score
================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :rectbot_cv/PoseObject "f36e3669b0713547250eac0a5f98a3b6")


