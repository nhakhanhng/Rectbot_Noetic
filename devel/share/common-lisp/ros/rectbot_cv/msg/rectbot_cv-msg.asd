
(cl:in-package :asdf)

(defsystem "rectbot_cv-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
               :vision_msgs-msg
)
  :components ((:file "_package")
    (:file "KeyPoint" :depends-on ("_package_KeyPoint"))
    (:file "_package_KeyPoint" :depends-on ("_package"))
    (:file "PoseObject" :depends-on ("_package_PoseObject"))
    (:file "_package_PoseObject" :depends-on ("_package"))
    (:file "PoseObjectArray" :depends-on ("_package_PoseObjectArray"))
    (:file "_package_PoseObjectArray" :depends-on ("_package"))
  ))