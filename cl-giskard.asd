; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cutplan
  :name "cl-giskard"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Wrapper giskard->lisp."
  :depends-on (:cl
               :cl-tf
               :cl-tf2
               :cl-transforms-stamped
               :giskard_msgs-msg
               :giskard_msgs-srv
               :giskard_msgs-act
               :ros-load-manifest
               :roslisp-utilities
               :roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "cl-giskard" :depends-on ("package"))))))
