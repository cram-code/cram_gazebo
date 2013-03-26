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

(defgeneric make-new-desig-description (old-desig perceived-object &key use-simple-knowledge)
  (:documentation "Merges the description of `old-desig' with the
properties of `perceived-object'.")
  (:method ((old-desig object-designator)
            (perceived-object object-designator-data)
            &key use-simple-knowledge)
    (let ((obj-loc-desig (make-designator
                          'location
                          `((pose ,(object-pose perceived-object))))))
      (cond (use-simple-knowledge
             (with-vars-strictly-bound (?handles ?min-handles)
                 (lazy-car
                  (prolog `(and (simple-knowledge:gazebo-object ?_ ?name ?_)
                                (simple-knowledge:object-handles
                                 ?name ?handles)
                                (simple-knowledge:object-min-handles
                                 ?name ?min-handles))
                          `(,@(when (object-identifier perceived-object)
                                `((?name . ,(object-identifier
                                             perceived-object)))))))
               `((at ,obj-loc-desig) (type ,(object-type perceived-object))
                 ,@(unless (member 'name (description old-desig) :key #'car)
                     `((name ,(object-identifier perceived-object))))
                 ,@(when ?min-handles
                     (unless (member 'min-handles (description old-desig)
                                     :key #'car)
                       `((min-handles ,?min-handles))))
                 ,@(when ?handles
                     (make-handle-designator-sequence ?handles))
                 ,@(remove-if (lambda (element)
                                (member element '(at type handle)))
                              (description old-desig) :key #'car))))
            (t
             `((desig-props:at ,obj-loc-desig)
               ,@(remove-if (lambda (element)
                              (member element '(at type handle)))
                            (description old-desig) :key #'car)))))))

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

(defun find-object (&key object-name object-type use-simple-knowledge)
  "Finds objects based on either their name `object-name' or their
type `object-type', depending what is given. An invalid combination of
both parameters will result in an empty list. When no parameters are
given, all known objects from the knowledge base are returned."
  (cond (use-simple-knowledge
         (mapcar (lambda (object)
                   (let* ((name (simple-knowledge:object-name object))
                          (filename (simple-knowledge:filename object))
                          (object-type (simple-knowledge:object-type object))
                          (model-pose (cram-gazebo-utilities:get-model-pose
                                       name :test #'object-names-equal)))
                     (when model-pose
                       (geometry->designator-data
                        name model-pose object-type
                        (get-object-geometry filename)
                        (get-object-geometry-pose filename)))))
                 (force-ll (lazy-mapcar (lambda (bindings)
                                          (var-value '?object bindings))
                                        (crs:prolog
                                         `(simple-knowledge:gazebo-object
                                           ?object
                                           ,(cond (object-name object-name)
                                                  (t '?name))
                                           ,(cond (object-type object-type)
                                                  (t '?type))))))))
        (t
         (let* ((model-pose (cram-gazebo-utilities:get-model-pose
                             object-name :test #'object-names-equal)))
           (when model-pose
             (list (make-instance 'gazebo-designator-shape-data
                                  :object-identifier object-name
                                  :pose model-pose)))))))

(defun perceived-object->designator (designator perceived-object)
  (make-effective-designator
   designator
   :new-properties (make-new-desig-description designator perceived-object)
   :data-object perceived-object))

(defun find-with-designator (designator)
  ;; TODO(moesenle): add verification of location using the AT
  ;; property.
  (with-desig-props (name type) designator
    (mapcar (lambda (perceived-object)
                (perceived-object->designator
                 designator perceived-object))
            (find-object :object-name name :object-type type))))

(defun emit-perception-event (designator)
  (cram-plan-knowledge:on-event (make-instance
                                 'cram-plan-knowledge:object-perceived-event
                                 :perception-source :gazebo-perception-process-module
                                 :object-designator designator))
  designator)

(def-process-module gazebo-perception-process-module (input)
  (assert (typep input 'action-designator))
  (let ((object-designator (reference input)))
    (ros-info (gazebo-perception-process-module process-module)
              "Searching for object ~a" object-designator)
    (let ((result (find-with-designator object-designator)))
      (unless result
        (fail 'object-not-found :object-desig input))
      (ros-info (gazebo-perception-process-module process-module)
                "Found objects: ~a" result)
      (map 'nil #'emit-perception-event result)
      result)))
