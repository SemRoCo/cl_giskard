; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cl-giskard
  :name "cl-giskard"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Wrapper giskard->lisp."
  :depends-on (:cl-tf
               :cl-tf2
               :cl-transforms-stamped
               :giskard_msgs-msg
               :giskard_msgs-srv
               :actionlib-lisp
               :ros-load-manifest
               :roslisp-utilities
               :roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "cl-giskard" :depends-on ("package"))))))
