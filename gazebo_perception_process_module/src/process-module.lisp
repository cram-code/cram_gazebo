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

(in-package :gazebo-perception-process-module)

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

(defun get-model-pose (name)
  "Return the current pose of a model with the name `name' spawned in
gazebo. The pose is given in the `map' frame."
  (cram-language:wait-for (cram-language:pulsed *model-state-msg*))
  (let ((model-state-msg (cram-language:value *model-state-msg*)))
    (when model-state-msg
      (with-fields
          ((name-sequence name)
           (pose-sequence pose)
           (twist-sequence twist))
          model-state-msg
        (let ((model-name-index (position name
                                          name-sequence
                                          :test #'equal)))
          (when model-name-index
            (tf:pose->pose-stamped
             "map"
             (roslisp:ros-time)
             (tf:msg->pose
              (elt pose-sequence model-name-index)))))))))

(defun model-state-callback (msg)
  "This is the callback for the gazebo topic subscriber subscribed on
`/gazebo/model_states'. It takes message `msg' with the format
`gazebo_msgs/ModelStates' as a parameter."
  (setf (cram-language:value *model-state-msg*) msg)
  (cram-language:pulse *model-state-msg*))

(def-process-module gazebo-perception-process-module (desig)
  "Definition of the gazebo-perception-process-module."
  (let ((newest-valid-designator (desig:newest-valid-designator input)))
    (or
     (mapcar (lambda (designator)
               (cram-plan-knowledge:on-event
                (make-instance 'cram-plan-knowledge:object-perceived-event
                               :perception-source :projection
                               :object-designator designator))
               designator)
             (if newest-valid-designator
                 (find-with-bound-designator newest-valid-designator)
                 (find-with-new-designator input)))
     (cpl:fail 'cram-plan-failures:object-not-found
               :object-desig input))))

(defclass projection-object-designator (desig:object-designator)
  ())

(defclass perceived-object (desig:object-designator-data)
  ((designator :reader object-designator :initarg :designator)))

(defmethod desig:designator-pose ((designator projection-object-designator))
  (desig:object-pose (desig:reference designator)))

(defmethod desig:designator-distance ((designator-1 desig:object-designator)
                                      (designator-2 desig:object-designator))
  (cl-transforms:v-dist (cl-transforms:origin (desig:designator-pose designator-1))
                        (cl-transforms:origin (desig:designator-pose designator-2))))

(defun make-object-designator (perceived-object &key parent type name)
  (let* ((pose (desig:object-pose perceived-object))
         (designator (change-class
                      (desig:make-designator
                       'desig-props:object
                       (desig:update-designator-properties
                        `(,@(when type `((desig-props:type ,type)))
                            (desig-props:at ,(desig:make-designator
                                              'desig:location `((desig-props:pose ,pose))))
                            ,@(when name `((desig-props:name ,name))))
                        (when parent (desig:properties parent)))
                       parent)
                      'projection-object-designator)))
    (setf (slot-value perceived-object 'designator) designator)
    (setf (slot-value designator 'desig:data) perceived-object)
    (setf (slot-value designator 'desig:valid) t)
    designator))

(defun find-object-with-id (id &key name type)
  (let ((obj-desig (gazebo-perception-process-module::make-object-designator
                    (make-instance 'gazebo-perception-pm::perceived-object
                                   :object-identifier id
                                   :pose nil)
                    :name name
                    :type type)))
    (find-object obj-desig)))

(defun find-object (designator)
  "Finds objects with (optional) name `object-name' and type `type'
and returns a list of elements of the form \(name pose\)."
  (let ((object-name (when (slot-value designator 'desig:data)
                       (desig:object-identifier (desig:reference designator))))
        (type (or (desig:desig-prop-value designator 'desig-props:type)
                  '?_)))
    (list object-name (get-model-pose object-name))))

(defun find-with-bound-designator (designator)
  (flet ((make-designator (object pose)
           (make-object-designator
            (make-instance
             'perceived-object
             :object-identifier object
             :pose pose)
            :name object
            :parent designator)))
    (cut:force-ll
     (cut:lazy-mapcar
      (alexandria:curry #'apply #'make-designator) (find-object designator)))))

(defun find-with-new-designator (designator)
  (desig:with-desig-props (desig-props:type) designator
    (flet ((make-designator (object pose)
             (make-object-designator
              (make-instance
               'perceived-object
               :object-identifier object
               :pose pose)
              :type desig-props:type
              :name object)))
      (when desig-props:type
        (cut:force-ll
         (cut:lazy-mapcar
          (alexandria:curry #'apply #'make-designator) (find-object designator)))))))

(cram-roslisp-common:register-ros-init-function init-gazebo-perception-process-module)