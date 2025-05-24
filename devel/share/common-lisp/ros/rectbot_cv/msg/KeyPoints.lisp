; Auto-generated. Do not edit!


(cl:in-package rectbot_cv-msg)


;//! \htmlinclude KeyPoints.msg.html

(cl:defclass <KeyPoints> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (score
    :reader score
    :initarg :score
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass KeyPoints (<KeyPoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyPoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyPoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rectbot_cv-msg:<KeyPoints> is deprecated: use rectbot_cv-msg:KeyPoints instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <KeyPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rectbot_cv-msg:point-val is deprecated.  Use rectbot_cv-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <KeyPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rectbot_cv-msg:score-val is deprecated.  Use rectbot_cv-msg:score instead.")
  (score m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyPoints>) ostream)
  "Serializes a message object of type '<KeyPoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'score) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyPoints>) istream)
  "Deserializes a message object of type '<KeyPoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'score) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyPoints>)))
  "Returns string type for a message object of type '<KeyPoints>"
  "rectbot_cv/KeyPoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyPoints)))
  "Returns string type for a message object of type 'KeyPoints"
  "rectbot_cv/KeyPoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyPoints>)))
  "Returns md5sum for a message object of type '<KeyPoints>"
  "92f94618554fe5449a29ca0200149d32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyPoints)))
  "Returns md5sum for a message object of type 'KeyPoints"
  "92f94618554fe5449a29ca0200149d32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyPoints>)))
  "Returns full string definition for message of type '<KeyPoints>"
  (cl:format cl:nil "geometry_msgs/Point32 point~%std_msgs/Float32 score~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyPoints)))
  "Returns full string definition for message of type 'KeyPoints"
  (cl:format cl:nil "geometry_msgs/Point32 point~%std_msgs/Float32 score~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyPoints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'score))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyPoints>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyPoints
    (cl:cons ':point (point msg))
    (cl:cons ':score (score msg))
))
