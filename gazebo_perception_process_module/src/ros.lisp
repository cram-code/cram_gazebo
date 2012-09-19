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

(in-package :gazebo-perception-process-module)

(defvar *gazebo-modelstates-subscriber* nil)
(defvar *model-state-msg* (cram-language:make-fluent :name :model-state-msg))

(defun init-gazebo-perception-process-module ()
  "Initialize the gazebo perception process module. At the moment,
this means subscribing on the gazebo model_states topic to be informed
about the current state of all models in the simulated world."
  (setf *gazebo-modelstates-subscriber*
        (subscribe
         "/gazebo/model_states"
         "gazebo_msgs/ModelStates"
         #'model-state-callback)))

(cram-roslisp-common:register-ros-init-function init-gazebo-perception-process-module)

(defun model-state-callback (msg)
  "This is the callback for the gazebo topic subscriber subscribed on
`/gazebo/model_states'. It takes message `msg' with the format
`gazebo_msgs/ModelStates' as a parameter."
  (setf (cram-language:value *model-state-msg*) msg)
  (cram-language:pulse *model-state-msg*))

(defun get-model-pose (name &key (test #'equal))
  "Return the current pose of a model with the name `name' spawned in
gazebo. The pose is given in the `map' frame."
  (cram-language:wait-for (cram-language:pulsed *model-state-msg*))
  (let ((model-state-msg (cram-language:value *model-state-msg*)))
    (when model-state-msg
      (with-fields ((name-sequence name)
                    (pose-sequence pose))
          model-state-msg
        (let ((model-name-index (position name name-sequence :test test)))
          (when model-name-index
            (tf:pose->pose-stamped
             "map" 0.0
             (tf:msg->pose
              (elt pose-sequence model-name-index)))))))))
