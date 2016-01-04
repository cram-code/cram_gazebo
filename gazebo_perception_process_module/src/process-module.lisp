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

(defmethod designator-pose ((designator object-designator))
  (object-pose (reference designator)))

(defmethod designator-distance ((designator-1 object-designator)
                                (designator-2 object-designator))
  (cl-transforms:v-dist (cl-transforms:origin (designator-pose designator-1))
                        (cl-transforms:origin (designator-pose designator-2))))

(defgeneric make-new-desig-description (old-desig perceived-object)
  (:documentation "Merges the description of `old-desig' with the
properties of `perceived-object'.")
  (:method ((old-desig object-designator)
            (perceived-object object-designator-data))
    (let ((obj-loc-desig (make-designator
                          'location
                          `((pose ,(object-pose perceived-object)))))
          (object-name (or (when (desig-prop-value old-desig 'desig-props:name)
                             (desig-prop-value old-desig 'desig-props:name))
                           (object-identifier perceived-object))))
      `((desig-props:at ,obj-loc-desig)
        (desig-props:name ,object-name)
        ,@(remove-if (lambda (element)
                       (member element '(at type name)))
                     (description old-desig) :key #'car)))))

(defun make-handle-designator-sequence (handles)
  "Converts the sequence `handles' (handle-pose handle-radius) into a
sequence of object designators representing handle objects. Each
handle object then consist of a location designator describing its
relative position as well as the handle's radius for grasping
purposes."
  (mapcar (lambda (handle-desc)
            (destructuring-bind (pose radius) handle-desc
              `(handle
                ,(make-designator 'object
                                  `((at ,(make-designator
                                          'location `((pose ,pose))))
                                    (radius ,radius)
                                    (type handle))))))
          handles))

(defun find-object (&key object-name object-type)
  "Finds objects based on either their name `object-name' or their
type `object-type', depending what is given. An invalid combination of
both parameters will result in an empty list. When no parameters are
given, all known objects from the knowledge base are returned."
  (cond (object-name
         (let* ((obj-symbol object-name)
                (model-pose (cram-gazebo-utilities:get-model-pose
                             object-name)))
           (when model-pose
             (list (make-instance 'gazebo-designator-shape-data
                                  :object-identifier obj-symbol
                                  :pose model-pose)))))
        (object-type
         (loop for model-data in (cram-gazebo-utilities:get-models)
               as name = (car model-data)
               when (string= object-type (subseq name 0 (length object-type)))
                 collect
                 (make-instance 'gazebo-designator-shape-data
                                :object-identifier name
                                :pose (cdr model-data))))
        (t
         (mapcar (lambda (model-data)
                   (destructuring-bind (model-name . model-pose)
                       model-data
                     (make-instance 'gazebo-designator-shape-data
                                    :object-identifier model-name
                                    :pose model-pose)))
                 (cram-gazebo-utilities:get-models)))))

(defun perceived-object->designator (designator perceived-object)
  (make-effective-designator
   designator
   :new-properties (make-new-desig-description
                    designator perceived-object)
   :data-object perceived-object))

(defun find-with-designator (designator)
  (with-desig-props (desig-props::name desig-props::type) designator
    (let* ((at (desig-prop-value designator 'desig-props::at))
           (filter-function
             (cond (at (lambda (object-check)
                         (let* ((sample (reference at))
                                ;; This is a 2d comparison; put the z
                                ;; coordinate from the sample into the
                                ;; pose before validating. Otherwise,
                                ;; gravity will mess up everything.
                                (pose (desig-prop-value
                                       (desig-prop-value
                                        object-check
                                        'desig-props::at)
                                       'desig-props::pose))
                                (pose-elevated
                                  (tf:copy-pose
                                   pose
                                   :origin (tf:make-3d-vector (tf:x (tf:origin pose))
                                                              (tf:y (tf:origin pose))
                                                              (tf:z (tf:origin sample))))))
                           (not (validate-location-designator-solution at pose-elevated)))))
                   (t #'not))))
      (remove-if
       filter-function
       (mapcar (lambda (perceived-object)
                 (perceived-object->designator
                  designator perceived-object))
               (find-object :object-name desig-props::name
                            :object-type desig-props::type))))))

(def-process-module gazebo-perception-process-module (input)
  (assert (typep input 'action-designator))
  (let ((object-designator (desig-prop-value input 'desig-props::obj)))
    (ros-info (gazebo perception-process-module)
              "Searching for object ~a" object-designator)
    (cram-task-knowledge:filter-perceived-objects
     object-designator
     (find-with-designator object-designator))))
