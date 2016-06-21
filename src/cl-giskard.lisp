;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.com>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :cl-giskard)

(defparameter *base-frame* "/map")
(defparameter *left-goal-frame* "/l_wrist_roll_link")
(defparameter *left-right-frame* "/r_wrist_roll_link")

(defconstant *giskard-command-topic-name* "/pr2_controler/goal")
(defconstant *giskard-command-topic-type* "giskard_msgs/WholeBodyCommand")

(defconstant *giskard-feedback-topic-name* "/pr2_controler/feedback")
(defconstant *giskard-feedback-topic-type* "giskard_msgs/ControllerFeedback")

(defconstant *giskard-state-topic-name* "/state")
(defconstant *giskard-state-topic-type* "giskard_msgs/WholeBodyState")

(defparameter *pub-giskard-goal* nil)

(defparameter *giskard-goal-l-ee* nil)
(defparameter *giskard-goal-r-ee* nil)

(defparameter *tf-listener* nil)

(defun ensure-tf-listener ()
  (if *tf-listener*
    *tf-listener*
    (setf *tf-listener* (make-instance 'cl-tf:transform-listener))))

(defun ensure-goal-mirror (arm)
  (cond
    ((not (typep arm 'string))
      (error "ensure-goal-mirror expects the type of the arm parameter to be string; instead got ~S" (type-of arm)))
    ((equal (string-downcase arm) "left")
      (if *giskard-goal-l-ee*
        *giskard-goal-l-ee*
        (cl-tf:lookup-transform (ensure-tf-listener) *base-frame* *left-goal-frame*)))
    ((equal (string-downcase arm) "right")
      (if *giskard-goal-l-ee*
        *giskard-goal-l-ee*
        (cl-tf:lookup-transform (ensure-tf-listener) *base-frame* *right-goal-frame*))))
    (T
      (error "ensure-goal-mirror expects the arm parameter to be left or right; instead got ~S" arm))))

(defun ps->msg (pose-stamped)
  (cl-transforms-stamped:make-pose-stamped-msg (cl-transforms-stamped:pose pose-stamped)
                                               (cl-transforms-stamped:frame-id pose-stamped)
                                               (cl-transforms-stamped:stamp pose-stamped)))

(defun ensure-pose-stamped-msg (pose-object)
  (cond
    ((typep pose-object 'geometry_msgs-msg:Pose)
      pose-object)
    ((typep pose-object 'cl-transforms-stamped:pose-stamped)
      (ps->msg pose-object))
    ((typep pose-object 'cl-transforms-stamped:transform-stamped)
      (ps->msg (cl-transforms-stamped:transform-stamped->pose-stamped pose-object)))
    ((typep pose-object 'cl-transforms-stamped:pose)
      (ps->msg (cl-transforms-stamped:make-pose-stamped *base-frame*
                                                        0.0
                                                        (cl-transforms-stamped:translation pose-object)
                                                        (cl-transforms-stamped:rotation pose-object))))
    ((typep pose-object 'cl-transforms-stamped:transform)
      (ps->msg (cl-transforms-stamped:make-pose-stamped *base-frame*
                                                        0.0
                                                        (cl-transforms-stamped:translation pose-object)
                                                        (cl-transforms-stamped:rotation pose-object))))
    (T
      (error "Object passed to ensure-pose-stamped-msg is not of a type that can be converted to geometry_msgs/PoseStamped."))))

(defun ensure-goal-publisher ()
  (if *pub-giskard-goal*
    *pub-giskard-goal*
    (progn
      (setf *pub-giskard-goal* (roslisp:advertise *giskard-command-topic-name* *giskard-command-topic-type* :latch nil))
      (roslisp:wait-duration 2.0)
      *pub-giskard-goal*)))

(defun send-two-arm-command (pose-left-ee pose-right-ee)
  "Takes two poses (OR NIL cl-transforms-stamped:pose cl-transforms-stamped:pose-stamped cl-transforms-stamped:transform cl-transforms-stamped:transform-stamped) and publishes them to the giskard controller command topic. If a pose is given as nil, then this function will send the previous non-NIL goal sent to that arm. If no non-NIL pose was sent to that arm, it will send the arm's current pose (so the arm should not move)."
  (let* ((left-ee-goal (if pose-left-ee 
                           (ensure-pose-stamped-msg pose-left-ee)
                           (ensure-goal-mirror "left")))
         (right-ee-goal (if pose-right-ee 
                            (ensure-pose-stamped-msg pose-right-ee)
                            (ensure-goal-mirror "right"))))
    (setf *giskard-goal-l-ee* left-ee-goal)
    (setf *giskard-goal-r-ee* right-ee-goal)
    (roslisp:publish (ensure-goal-publisher)
                     (roslisp:make-message *giskard-command-topic-type*
                                           :left_ee_goal left-ee-goal
                                           :right_ee_goal right-ee-goal))))

(defun send-left-arm-command (pose-left-ee)
  "A convenience function to send a goal (OR NIL cl-transforms-stamped:pose cl-transforms-stamped:pose-stamped cl-transforms-stamped:transform cl-transforms-stamped:transform-stamped) just to the left arm. The right arm will continue to do what it was doing (following the previous goal, or staying put if there was no previous goal)."
  (let* ((pose-right-ee (ensure-goal-mirror "right")))
    (send-two-arm-command pose-left-ee pose-right-ee)))

(defun send-right-arm-command (pose-right-ee)
  "A convenience function to send a goal (OR NIL cl-transforms-stamped:pose cl-transforms-stamped:pose-stamped cl-transforms-stamped:transform cl-transforms-stamped:transform-stamped) just to the right arm. The right arm will continue to do what it was doing (following the previous goal, or staying put if there was no previous goal)."
  (let* ((pose-left-ee (ensure-goal-mirror "left")))
    (send-two-arm-command pose-left-ee pose-right-ee)))

(defun setup-feedback-listener (callback-fn &key (max-queue-length 'infty))
  "Configures callback-fn to be called whenever the giskard controller feedback topic is updated."
  (roslisp:subscribe *giskard-feedback-topic-name* *giskard-feedback-topic-type* callback-fn :max-queue-length max-queue-length))


