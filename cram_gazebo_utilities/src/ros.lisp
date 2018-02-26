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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(defvar *gazebo-modelstates-subscriber* nil)
(defvar *known-models* (make-hash-table :test 'equal))
(defvar *models-lock* (make-lock :name "Known Gazebo models"))

(defun init-cram-gazebo-utilities ()
  "Initialize the cram gazebo utilities. At the moment,
this means subscribing to the gazebo model_states topic to be informed
about the current state of all models in the simulated world."
  (setf *gazebo-modelstates-subscriber*
        (subscribe
         "/gazebo/model_states"
         "gazebo_msgs/ModelStates"
         #'model-state-callback)))

(roslisp-utilities:register-ros-init-function init-cram-gazebo-utilities)

(defun model-state-callback (msg)
  "This is the callback for the gazebo topic subscriber subscribed to `/gazebo/model_states'. It takes message `msg' with the format `gazebo_msgs/ModelStates' as a parameter."
  (with-lock-held (*models-lock*)
    (clrhash *known-models*)
    (with-fields ((names name)
                  (poses pose))
        msg
      (map 'nil (lambda (model-name model-pose-msg)
                  (setf (gethash model-name *known-models*)
                        (cl-transforms-stamped:pose->pose-stamped
                         "map" 0.0
                         (cl-transforms-stamped:from-msg model-pose-msg))))
           names poses))))

(defun get-model-pose (name)
  "Return the current pose of a model with the name `name' spawned in Gazebo. The pose is given in the `map' frame."
  (with-lock-held (*models-lock*)
    (gethash name *known-models*)))

(defun get-models ()
  "Returns a list of cons elements containing `(model-name . model-pose)' of all currently known models in Gazebo."
  (with-lock-held (*models-lock*)
    (loop for hash-key being the hash-keys of *known-models*
          collect (cons hash-key (gethash hash-key *known-models*)))))
