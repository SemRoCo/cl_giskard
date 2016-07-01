; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cl-giskard
  :name "cl-giskard"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Wrapper giskard->lisp."
  :depends-on (:giskard_msgs-msg
               :giskard_msgs-srv
               :geometry_msgs-msg
               :cl-transforms-stamped
               :actionlib-lisp
               :roslisp
               :roslisp-utilities)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "cl-giskard" :depends-on ("package"))))))
