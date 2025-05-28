; Auto-generated. Do not edit!


(cl:in-package rectbot_cv-msg)


;//! \htmlinclude KeyPoint.msg.html

(cl:defclass <KeyPoint> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (y
    :reader y
    :initarg :y
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (score
    :reader score
    :initarg :score
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass KeyPoint (<KeyPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rectbot_cv-msg:<KeyPoint> is deprecated: use rectbot_cv-msg:KeyPoint instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <KeyPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rectbot_cv-msg:x-val is deprecated.  Use rectbot_cv-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <KeyPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rectbot_cv-msg:y-val is deprecated.  Use rectbot_cv-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <KeyPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rectbot_cv-msg:score-val is deprecated.  Use rectbot_cv-msg:score instead.")
  (score m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyPoint>) ostream)
  "Serializes a message object of type '<KeyPoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'x) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'y) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'score) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyPoint>) istream)
  "Deserializes a message object of type '<KeyPoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'x) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'y) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'score) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyPoint>)))
  "Returns string type for a message object of type '<KeyPoint>"
  "rectbot_cv/KeyPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyPoint)))
  "Returns string type for a message object of type 'KeyPoint"
  "rectbot_cv/KeyPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyPoint>)))
  "Returns md5sum for a message object of type '<KeyPoint>"
  "1cd825e8df87066e1486d6a8d6cfd264")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyPoint)))
  "Returns md5sum for a message object of type 'KeyPoint"
  "1cd825e8df87066e1486d6a8d6cfd264")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyPoint>)))
  "Returns full string definition for message of type '<KeyPoint>"
  (cl:format cl:nil "std_msgs/Float32 x~%std_msgs/Float32 y~%std_msgs/Float32 score~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyPoint)))
  "Returns full string definition for message of type 'KeyPoint"
  (cl:format cl:nil "std_msgs/Float32 x~%std_msgs/Float32 y~%std_msgs/Float32 score~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyPoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'x))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'y))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'score))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyPoint
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':score (score msg))
))
