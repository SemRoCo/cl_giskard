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

(defparameter *tf-default-timeout* 2.0)

(defparameter *left-arm-converged* nil)
(defparameter *right-arm-converged* nil)

(defparameter *feedback-mutex* (sb-thread:make-mutex))

(defun ensure-goal-object (goal-object)
  (typecase goal-object
    (list
     goal-object)
    (vector
     goal-object)
    (geometry_msgs-msg:PoseStamped
     goal-object)
    (cl-transforms-stamped:pose-stamped
     (cl-transforms-stamped:to-msg goal-object))
    (cl-transforms-stamped:transform-stamped
     (cl-transforms-stamped:to-msg
      (cl-transforms-stamped:transform-stamped->pose-stamped goal-object)))
    (cl-transforms-stamped:pose
     (cl-transforms-stamped:to-msg
      (cl-transforms-stamped:make-pose-stamped
       *base-frame*
       0.0
       (cl-transforms-stamped:origin goal-object)
       (cl-transforms-stamped:orientation goal-object))))
    (cl-transforms-stamped:transform
     (cl-transforms-stamped:to-msg
      (cl-transforms-stamped:make-pose-stamped
       *base-frame*
       0.0
       (cl-transforms-stamped:translation goal-object)
       (cl-transforms-stamped:rotation goal-object))))
    (T
     (error "Object passed to ensure-goal-object is not of a type that can be converted to geometry_msgs/PoseStamped."))))

;;;; Action-interface to the giskard controller.
;;;; 

;;(defun get-left-arm-transform (tf-listener)
;;  (cl-transforms-stamped:lookup-transform
;;   tf-listener
;;   *base-frame*
;;   *left-goal-frame*
;;   :timeout *tf-default-timeout*))

;;(defun get-right-arm-transform (tf-listener)
;;  (cl-transforms-stamped:lookup-transform
;;   tf-listener
;;   *base-frame*
;;   *right-goal-frame*
;;   :timeout *tf-default-timeout*))

(defun ensure-giskard-action-client ()
  (unless *giskard-action-client*
    (setf *giskard-action-client*
          (actionlib-lisp:make-simple-action-client
           *giskard-action-server-name*
           *giskard-action-server-type*))
    (loop until (actionlib-lisp:wait-for-server *giskard-action-client* 5.0)
          do (roslisp:ros-info (cl-giskard) "Waiting for giskard action server."))
    (roslisp:ros-info (cl-giskard) "Action client created.")
    (roslisp:wait-duration 1.0)) ; somehow wait-for-server doesn't help here
  *giskard-action-client*)

(defun destroy-giskard-action-client ()
  (setf *giskard-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-giskard-action-client)
;; if a node is restarted and the action client is not released things stop working

(defun cancel-action-goal ()
  (actionlib-lisp:cancel-goal (ensure-giskard-action-client)))

(defun action-goal-status ()
  (actionlib-lisp:state (ensure-giskard-action-client)))

(defun action-goal-result ()
  (actionlib-lisp:result (ensure-giskard-action-client)))

(defun wait-for-action-result (&optional (timeout 0))
  (actionlib-lisp:wait-for-result (ensure-giskard-action-client) timeout))

(defun left-arm-converged ()
  (sb-thread:with-mutex (*feedback-mutex*)
    *left-arm-converged*))

(defun right-arm-converged ()
  (sb-thread:with-mutex (*feedback-mutex*)
    *right-arm-converged*))

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

(defun send-yaml-action (yaml thresholds
                         &key
                           (feedback-cb (lambda (feedback-msg)
                                          (declare (ignore feedback-msg))))
                           (done-cb (lambda (status result)
                                      (declare (ignore status result))))
                           (active-cb (lambda () )))
  "Takes a yaml string and a list of string-value pairs and sends them
  to the giskard action server."
  (let* ((thresholds (mapcar (lambda (threshold)
                               (let* ((semantics (car threshold))
                                      (value (cadr threshold)))
                                 (roslisp:make-message "giskard_msgs/SemanticFloat64"
                                                       :semantics semantics
                                                       :value value)))
                             thresholds)))
    (actionlib-lisp:send-goal
      (ensure-giskard-action-client)
      (actionlib-lisp:make-action-goal-msg
        (ensure-giskard-action-client)
        command
          (roslisp:make-message *giskard-action-goal-type*
                               :type 1 ;; Use the YAML_CONTROLLER interface
                               :yaml_spec yaml
                               :convergence_thresholds (coerce thresholds 'vector)))
      :feedback-cb (lambda (feedback-msg)
                    (let* ((feedback (msg->feedback feedback-msg)))
                      ;; this is to extract information about whether the arms have converged
                      (roslisp:with-fields ((left-arm-moving (left_arm_moving state))
                                            (torso-moving (torso_moving state))
                                            (right-arm-moving (right_arm_moving state))) feedback-msg
                        (sb-thread:with-mutex (*feedback-mutex*)
                          (setf *left-arm-converged* (and (not left-arm-moving) (not torso-moving)))
                          (setf *right-arm-converged* (and (not right-arm-moving) (not torso-moving)))))
                      (apply feedback-cb (list feedback))))
      :done-cb (lambda (status result-msg)
                (let* ((result (msg->result result-msg)))
                  (apply done-cb (list status result))
                  ;; this is to prevent the actionlib client from sending a done-cb indefinitely
                  (actionlib-lisp:send-goal (ensure-giskard-action-client)
                                            (actionlib-lisp:make-action-goal-msg (ensure-giskard-action-client)
                                                                                 command
                                                                                 (roslisp:make-message *giskard-action-goal-type*
                                                                                                       :type 1
                                                                                                       :yaml_spec yaml
                                                                                                       :convergence_thresholds (coerce thresholds 'vector))))))
      :active-cb active-cb)))

(defun send-action-goal (goal-left-ee goal-right-ee
                         &key
                           (feedback-cb (lambda (feedback-msg)
                                          (declare (ignore feedback-msg))))
                           (done-cb (lambda (status result)
                                      (declare (ignore status result))))
                           (active-cb (lambda () )))
  (declare (type (or null
                     list
                     vector
                     cl-transforms-stamped:pose
                     cl-transforms-stamped:pose-stamped
                     cl-transforms-stamped:transform
                     cl-transforms-stamped:transform-stamped)
                 goal-left-ee goal-right-ee))
  "Takes two poses or lists of numbers and publishes them to the giskard controller action server.
If a pose is given as nil, then the corresponding arm will keep doing
what it was doing before this function was called (and if it has no previous goals, it will stay put).

feedback-cb should be either left unset or a (lambda (msg) ..)
  where msg is of type giskard_msgs-msg:WholeBodyState.
done-cb should be either left unset or a (lambda (status msg) ..)
  where status is an actionlib-lisp status and msg is of type giskard_msgs-msg:WholeBodyState.
active-cb should be left unset or a (lambda () ..)."
  (let* ((goal-left-ee (ensure-goal-object goal-left-ee))
         (goal-right-ee (ensure-goal-object goal-right-ee))
         (left-ee-goal (if goal-left-ee
                         (if (typep goal-left-ee 'geometry_msgs-msg:PoseStamped)
                           (roslisp:make-message
                            *giskard-action-goal-part-type*
                            :type 1 ;; CARTESIAN_GOAL
                            :goal_pose goal-left-ee)
                           (roslisp:make-message
                             *giskard-action-goal-part-type*
                             :type 2 ;; JOINT_GOAL
                             :goal_configuration (coerce goal-left-ee 'vector)))
                         (roslisp:make-message *giskard-action-goal-part-type*)))
         (right-ee-goal (if goal-right-ee
                          (if (typep goal-right-ee 'geometry_msgs-msg:PoseStamped)
                            (roslisp:make-message
                             *giskard-action-goal-part-type*
                             :type 1 ;; CARTESIAN_GOAL
                             :goal_pose goal-right-ee)
                            (roslisp:make-message
                             *giskard-action-goal-part-type*
                             :type 2 ;; JOINT_GOAL
                             :goal_configuration (coerce goal-right-ee 'vector)))
                          (roslisp:make-message *giskard-action-goal-part-type*))))
    (actionlib-lisp:send-goal
     (ensure-giskard-action-client)
     (actionlib-lisp:make-action-goal-msg
         (ensure-giskard-action-client)
       command
       (roslisp:make-message *giskard-action-goal-type*
                             :type 0 ;; Use the STANDARD_CONTROLLER interface
                             :left_ee left-ee-goal
                             :right_ee right-ee-goal))
     :feedback-cb (lambda (feedback-msg)
                    (let* ((feedback (msg->feedback feedback-msg)))
                      ;; this is to extract information about whether the arms have converged
                      (roslisp:with-fields ((left-arm-moving (left_arm_moving state))
                                            (torso-moving (torso_moving state))
                                            (right-arm-moving (right_arm_moving state))) feedback-msg
                        (sb-thread:with-mutex (*feedback-mutex*)
                          (setf *left-arm-converged* (and (not left-arm-moving) (not torso-moving)))
                          (setf *right-arm-converged* (and (not right-arm-moving) (not torso-moving)))))
                      (apply feedback-cb (list feedback))))
     :done-cb (lambda (status result-msg)
                (let* ((result (msg->result result-msg)))
                  (apply done-cb (list status result))
                  ;; this is to prevent the actionlib client from sending a done-cb indefinitely
                  (actionlib-lisp:send-goal (ensure-giskard-action-client)
                                            (actionlib-lisp:make-action-goal-msg (ensure-giskard-action-client)
                                                                                 command
                                                                                 (roslisp:make-message *giskard-action-goal-type*
                                                                                                       :left_ee left-ee-goal
                                                                                                       :right_ee right-ee-goal)))))
     :active-cb active-cb)))

(defun send-left-arm-action (goal-left-ee
                             &key
                               (feedback-cb (lambda (feedback-msg)
                                              (declare (ignore feedback-msg))))
                               (done-cb (lambda (status result)
                                          (declare (ignore status result))))
                               (active-cb (lambda () )))
  (send-action-goal goal-left-ee nil
                    :feedback-cb feedback-cb
                    :done-cb done-cb
                    :active-cb active-cb))

(defun send-right-arm-action (goal-right-ee
                              &key
                                (feedback-cb (lambda (feedback-msg)
                                               (declare (ignore feedback-msg))))
                                (done-cb (lambda (status result)
                                           (declare (ignore status result))))
                                (active-cb (lambda () )))
  (send-action-goal nil goal-right-ee
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
  (declare (type (or null
                     cl-transforms-stamped:pose
                     cl-transforms-stamped:pose-stamped
                     cl-transforms-stamped:transform
                     cl-transforms-stamped:transform-stamped)
                 pose-left-ee pose-right-ee))
  "Takes two poses and publishes them to the giskard controller command topic.
If a pose is given as nil, then this function will send the previous non-NIL goal sent to that arm
If no non-NIL pose was sent to that arm, it will send the arm's current pose (so the arm should not move)."
  (let* ((left-ee-goal (when pose-left-ee
                         (ensure-goal-object pose-left-ee)))
         (right-ee-goal (when pose-right-ee
                            (ensure-goal-object pose-right-ee)))
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
  (declare (type (or null
                     cl-transforms-stamped:pose
                     cl-transforms-stamped:pose-stamped
                     cl-transforms-stamped:transform
                     cl-transforms-stamped:transform-stamped)
                 pose-left-ee))
  "A convenience function to send a goal just to the left arm.
The right arm will continue to do what it was doing (following the previous goal,
or staying put if there was no previous goal)."
  (send-two-arm-command pose-left-ee nil))

(defun send-right-arm-command (pose-right-ee)
  (declare (type (or null
                     cl-transforms-stamped:pose
                     cl-transforms-stamped:pose-stamped
                     cl-transforms-stamped:transform
                     cl-transforms-stamped:transform-stamped)
                 pose-right-ee))
  "A convenience function to send a goal just to the right arm.
The right arm will continue to do what it was doing (following the previous goal,
or staying put if there was no previous goal)."
  (send-two-arm-command nil pose-right-ee))

(defun setup-feedback-listener (callback-fn &key (max-queue-length 'infty))
  "Configures callback-fn to be called whenever the giskard controller feedback topic is updated."
  (roslisp:subscribe *giskard-feedback-topic-name*
                     *giskard-feedback-topic-type*
                     callback-fn
                     :max-queue-length max-queue-length))

