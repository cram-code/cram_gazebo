;;; Copyright (c) 2012, Jan Winkler <winkler@cs.uni-bremen.de>
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

(in-package :cram-gazebo-utilities)

(defun set-model-state (model-name new-pose)
  (call-service "gazebo/set_model_state"
                'gazebo_msgs-srv:setmodelstate
                :model_state (make-msg "gazebo_msgs/ModelState"
                                       :model_name model-name
                                       :pose (tf:pose->msg new-pose)
                                       :reference_frame (tf:frame-id new-pose))))

(defun spawn-gazebo-model (name pose urdf-file)
  (call-service "gazebo/spawn_urdf_model"
                'gazebo_msgs-srv:spawnmodel
                :model_name name
                :model_xml (file-string urdf-file)
                :initial_pose (tf:pose->msg pose)
                :reference_frame (tf:frame-id pose)))

(defun delete-gazebo-model (name)
  (call-service "gazebo/delete_model"
                'gazebo_msgs-srv:deletemodel
                :model_name name))

(defun file-string (path)
  (with-open-file (s path)
    (let* ((len (file-length s))
           (data (make-string len)))
     (multiple-value-bind (string length) (values data (read-sequence data s))
        (declare (ignore length))
        string))))

(defun gazebo-present ()
  (roslisp:wait-for-service "gazebo/spawn_urdf_model" 0.25))

(defun set-joint-effort (joint effort &key (duration 2.0))
  (call-service "gazebo/apply_joint_effort"
                'gazebo_msgs-srv:applyjointeffort
                :joint_name joint
                :effort effort
                :start_time (roslisp:ros-time)
                :duration duration))

(defun set-joint-damping (joint damping)
  (call-service "gazebo/set_joint_properties"
                'gazebo_msgs-srv:setjointproperties
                :joint_name joint
                :ode_joint_config
                (roslisp:make-message
                 "gazebo_msgs/ODEJointProperties"
                 :damping (vector damping))))

(defun set-joint-position (model joint position &optional hold)
  (call-service "gazebo/joint_control"
                'attache_msgs-srv:jointcontrol
                :model model
                :joint joint
                :position position
                :hold_position hold))

(defun get-joint-information (model joint)
  (with-fields (success position min max)
      (call-service "gazebo/joint_information"
                    'attache_msgs-srv:jointinformation
                    :model model
                    :joint joint)
    (values success position min max)))

(defun open-joint (model joint &optional hold)
  (multiple-value-bind (success position min max)
      (get-joint-information model joint)
    (declare (ignore position min))
    (when success
      (set-joint-position model joint max hold))))

(defun close-joint (model joint &optional hold)
  (multiple-value-bind (success position min max)
      (get-joint-information model joint)
    (declare (ignore position max))
    (when success
      (set-joint-position model joint min hold))))
