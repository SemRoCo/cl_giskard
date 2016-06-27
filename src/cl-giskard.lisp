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

(defparameter *base-frame* "base_link")
(defparameter *left-goal-frame* "/l_gripper_tool_frame")
(defparameter *right-goal-frame* "/r_gripper_tool_frame")

(defparameter *giskard-command-topic-name* "/pr2_controller/goal")
(defparameter *giskard-command-topic-type* "giskard_msgs/WholeBodyCommand")
(defparameter *giskard-command-topic-part-type* "giskard_msgs/ArmCommand")

(defparameter *giskard-feedback-topic-name* "/pr2_controller/feedback")
(defparameter *giskard-feedback-topic-type* "giskard_msgs/ControllerFeedback")

(defparameter *pub-giskard-goal* nil)

(defparameter *giskard-action-client* nil)

(defparameter *giskard-action-server-name* "/controller_action_server/move")
(defparameter *giskard-action-server-type* "giskard_msgs/WholeBodyAction")
(defparameter *giskard-action-goal-part-type* "giskard_msgs/ArmCommand")
(defparameter *giskard-action-goal-type* "giskard_msgs/WholeBodyCommand")

(defparameter *tf-listener* nil)

(defun ensure-tf-listener ()
  (if *tf-listener*
      *tf-listener*
      (progn
        (setf *tf-listener* (make-instance 'cl-tf:transform-listener))
        (roslisp:wait-duration 1.0)
        *tf-listener*)))

(defun ensure-pose-stamped-msg (pose-object)
  (typecase pose-object
    (geometry_msgs-msg:PoseStamped
     pose-object)
    (cl-transforms-stamped:pose-stamped
     (cl-transforms-stamped:to-msg pose-object))
    (cl-transforms-stamped:transform-stamped
     (cl-transforms-stamped:to-msg
      (cl-transforms-stamped:transform-stamped->pose-stamped pose-object)))
    (cl-transforms-stamped:pose
     (cl-transforms-stamped:to-msg
      (cl-transforms-stamped:make-pose-stamped
       *base-frame*
       0.0
       (cl-transforms-stamped:translation pose-object)
       (cl-transforms-stamped:rotation pose-object))))
    (cl-transforms-stamped:transform
     (cl-transforms-stamped:to-msg
      (cl-transforms-stamped:make-pose-stamped
       *base-frame*
       0.0
       (cl-transforms-stamped:translation pose-object)
       (cl-transforms-stamped:rotation pose-object))))
    (T
     (error "Object passed to ensure-pose-stamped-msg is not of a type that can be converted to geometry_msgs/PoseStamped."))))

;;;; Action-interface to the giskard controller.
;;;; 

(defun get-left-arm-transform ()
  (cl-tf:lookup-transform (ensure-tf-listener) *base-frame* *left-goal-frame*))

(defun get-right-arm-transform ()
  (cl-tf:lookup-transform (ensure-tf-listener) *base-frame* *right-goal-frame*))

(defun ensure-giskard-action-client ()
  (if *giskard-action-client*
      *giskard-action-client*
      (progn
        (setf *giskard-action-client*
              (actionlib-lisp:make-simple-action-client
               *giskard-action-server-name*
               *giskard-action-server-type*))
        (actionlib-lisp:wait-for-server *giskard-action-client*)
        (roslisp:wait-duration 1.0)
        *giskard-action-client*)))

(defun cancel-action-goal ()
  (actionlib-lisp:cancel-goal (ensure-giskard-action-client)))

(defun action-goal-status ()
  (actionlib-lisp:state (ensure-giskard-action-client)))

(defun action-goal-result ()
  (actionlib-lisp:result (ensure-giskard-action-client)))

(defun wait-for-action-result (&optional (timeout 0))
  (actionlib-lisp:wait-for-result (ensure-giskard-action-client) timeout))

(defun left-arm-converged ()
  (let* ((result (action-goal-result)))
    (if result
        (roslisp:with-fields ((left-arm-converged left_arm_converged)) result
          left-arm-converged))))

(defun right-arm-converged ()
  (let* ((result (action-goal-result)))
    (if result
        (roslisp:with-fields ((right-arm-converged right_arm_converged)) result
          right-arm-converged))))

(defun arms-converged ()
  (and (left-arm-converged) (right-arm-converged)))

(defun msg->feedback (feedback-msg)
  ;; Perhaps in the future we'll want a Lisp structure to hold the data of a giskard feedback message,
  ;; but until then using the message itself is good enough.
  feedback-msg)

(defun msg->result (result-msg)
  ;; Perhaps in the future we'll want a Lisp structure to hold the data of a giskard feedback message,
  ;; but until then using the message itself is good enough.
  result-msg)

(defun send-action-goal (pose-left-ee pose-right-ee
                         &key
                           (feedback-cb (lambda (feedback-msg)
                                          (declare (ignore feedback-msg))))
                           (done-cb (lambda (status result)
                                      (declare (ignore status result))))
                           (active-cb (lambda () )))
  "Takes two poses (OR NIL cl-transforms-stamped:pose cl-transforms-stamped:pose-stamped
cl-transforms-stamped:transform cl-transforms-stamped:transform-stamped)
and publishes them to the giskard controller action server.
If a pose is given as nil, then the corresponding arm will keep doing
what it was doing before this function was called (and if it has no previous goals, it will stay put).

feedback-cb should be either left unset or a (lambda (msg) ..)
  where msg is of type giskard_msgs-msg:WholeBodyState.
done-cb should be either left unset or a (lambda (status msg) ..)
  where status is an actionlib-lisp status and msg is of type giskard_msgs-msg:WholeBodyState.
active-cb should be left unset or a (lambda () ..)."
  (let* ((left-ee-goal (if pose-left-ee
                           (roslisp:make-message
                            *giskard-action-goal-part-type*
                            :goal (ensure-pose-stamped-msg pose-left-ee)
                            :process T)
                           (roslisp:make-message *giskard-action-goal-part-type*)))
         (right-ee-goal (if pose-right-ee
                            (roslisp:make-message
                             *giskard-action-goal-part-type*
                             :goal (ensure-pose-stamped-msg pose-right-ee)
                             :process T)
                            (roslisp:make-message *giskard-action-goal-part-type*))))
    (actionlib-lisp:send-goal
     (ensure-giskard-action-client)
     (actionlib-lisp:make-action-goal-msg
         (ensure-giskard-action-client)
       command
       (roslisp:make-message *giskard-action-goal-type*
                             :left_ee left-ee-goal
                             :right_ee right-ee-goal))
     :feedback-cb (lambda (feedback-msg)
                    (let* ((feedback (msg->feedback feedback-msg)))
                      (apply feedback-cb (list feedback))))
     :done-cb (lambda (status result-msg)
                (let* ((result (msg->result result-msg)))
                  (apply done-cb (list status result))))
     :active-cb active-cb)))

(defun send-left-arm-action (pose-left-ee
                             &key
                               (feedback-cb (lambda (feedback-msg)
                                              (declare (ignore feedback-msg))))
                               (done-cb (lambda (status result)
                                          (declare (ignore status result))))
                               (active-cb (lambda () )))
  (send-action-goal pose-left-ee nil
                    :feedback-cb feedback-cb
                    :done-cb done-cb
                    :active-cb active-cb))

(defun send-right-arm-action (pose-right-ee
                              &key
                                (feedback-cb (lambda (feedback-msg)
                                               (declare (ignore feedback-msg))))
                                (done-cb (lambda (status result)
                                           (declare (ignore status result))))
                                (active-cb (lambda () )))
  (send-action-goal nil pose-right-ee
                    :feedback-cb feedback-cb
                    :done-cb done-cb
                    :active-cb active-cb))

;;;; Topic-interface to the giskard controller itself. These functions look like they should be exported by the package, but aren't.
;;;; This is because the topic interface to the controller itself is not actually intended for use by anyone except giskard.
;;;; These functions have been left here rather than deleted though because a) they took some work and polish and b) I could bet we'll need
;;;; that topic interface someday ;)
;;;; Still, until that day happens, don't use these functions, they are not maintained.

;;;; TODO: handle initializing goal mirror (currently done via a lookup of the transform from base_link to *_wrist_roll_link, but the eef may be in a different link)
;;;; TODO: add checks that goal poses are given relative to base_link; decide what to do in case they are not (a tf lookup is the obvious answer, but the reason
;;;; the giskard controller expects poses in a particular frame is to avoid tf-induced delays; lack of tf-delays would also be a reason why a wrapper to the topic
;;;; interface might be useful, so tf lookups should be avoided)

(defun ensure-goal-publisher ()
  (if *pub-giskard-goal*
      *pub-giskard-goal*
      (progn
        (setf *pub-giskard-goal*
              (roslisp:advertise *giskard-command-topic-name*
                                 *giskard-command-topic-type*
                                 :latch nil))
        (roslisp:wait-duration 2.0)
        *pub-giskard-goal*)))

(defun send-two-arm-command (pose-left-ee pose-right-ee)
  "Takes two poses (OR NIL cl-transforms-stamped:pose cl-transforms-stamped:pose-stamped
cl-transforms-stamped:transform cl-transforms-stamped:transform-stamped)
and publishes them to the giskard controller command topic.
If a pose is given as nil, then this function will send the previous non-NIL goal sent to that arm
If no non-NIL pose was sent to that arm, it will send the arm's current pose (so the arm should not move)."
  (let* ((left-ee-goal (if pose-left-ee 
                           (ensure-pose-stamped-msg pose-left-ee)
                           nil))
         (right-ee-goal (if pose-right-ee 
                            (ensure-pose-stamped-msg pose-right-ee)
                            nil))
         (left-ee (if left-ee-goal
                      (roslisp:make-message *giskard-command-topic-part-type*
                                            :goal left-ee-goal
                                            :process T)
                      (roslisp:make-message *giskard-command-topic-part-type*)))
         (right-ee (if right-ee-goal
                       (roslisp:make-message *giskard-command-topic-part-type*
                                             :goal right-ee-goal
                                             :process T)
                       (roslisp:make-message *giskard-command-topic-part-type*))))
    (roslisp:publish (ensure-goal-publisher)
                     (roslisp:make-message *giskard-command-topic-type*
                                           :left_ee left-ee
                                           :right_ee right-ee))))

(defun send-left-arm-command (pose-left-ee)
  "A convenience function to send a goal (OR NIL cl-transforms-stamped:pose cl-transforms-stamped:pose-stamped
cl-transforms-stamped:transform cl-transforms-stamped:transform-stamped) just to the left arm.
The right arm will continue to do what it was doing (following the previous goal,
or staying put if there was no previous goal)."
  (send-two-arm-command pose-left-ee nil))

(defun send-right-arm-command (pose-right-ee)
  "A convenience function to send a goal (OR NIL cl-transforms-stamped:pose
 cl-transforms-stamped:pose-stamped cl-transforms-stamped:transform cl-transforms-stamped:transform-stamped)
just to the right arm.
The right arm will continue to do what it was doing (following the previous goal,
or staying put if there was no previous goal)."
  (send-two-arm-command nil pose-right-ee))

(defun setup-feedback-listener (callback-fn &key (max-queue-length 'infty))
  "Configures callback-fn to be called whenever the giskard controller feedback topic is updated."
  (roslisp:subscribe *giskard-feedback-topic-name*
                     *giskard-feedback-topic-type*
                     callback-fn
                     :max-queue-length max-queue-length))

