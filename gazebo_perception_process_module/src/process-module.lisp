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

; Generate logs - copied from the robosherlock perception module...
(cut:define-hook cram-language::on-prepare-perception-request (designator-request))
(cut:define-hook cram-language::on-finish-perception-request (log-id designators-result))

; The following stuff is copied from cram-task-knowledge since it is not yet available in cram2...
(define-hook objects-perceived (object-template object-designators))

(defparameter *tf-listener* nil)
(defun ensure-tf-listener ()
  (unless *tf-listener*
    (progn
      (setf *tf-listener* (make-instance 'cl-tf:transform-listener))
      (roslisp:wait-duration 1.0)))
  *tf-listener*)

(defun destroy-tf-listener ()
  (setf *tf-listener* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-tf-listener)


(defgeneric filter-perceived-objects (object-template perceived-objects)
  (:documentation "Filters all perceived objects according to all registered filters. This method is mainly used by perception process modules that want to validate and filter their results. Also, this function triggers the `object-perceived-event' plan event, updating the belief state.")
  (:method (object-template perceived-objects)
    (let* ((filtered-objects
             (loop for filter-result in (objects-perceived
                                         object-template perceived-objects)
                   append filter-result)))
      filtered-objects)))

; Start of the original module
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
                          :location
                          `((:pose ,(object-pose perceived-object)))))
          (object-name (or (when (desig-prop-value old-desig :name)
                             (desig-prop-value old-desig :name))
                           (object-identifier perceived-object))))
      `((:at ,obj-loc-desig)
        (:name ,object-name)
        ,@(remove-if (lambda (element)
                       (member element '(:at type :name)))
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
                ,(make-designator :object
                                  `((:at ,(make-designator
                                          :location `((:pose ,pose))))
                                    (:radius ,radius)
                                    (:type handle))))))
          handles))

(defun filter-models-by-ignored-objects (model-names)
  (cpl:mapcar-clean (lambda (model-name)
                      (unless (find model-name *ignored-objects* :test #'string=)
                        model-name))
                    model-names))

(defun filter-models-by-name (model-names &key template-name)
  "If defined, `template-name' is the only valid model name returned (if present in `model-names'). Otherwise, `model-names' is returned."
  (cond (template-name
         (cpl:mapcar-clean (lambda (model-name)
                             (when (string= template-name model-name)
                               model-name))
                           model-names))
        (t model-names)))

(defun filter-models-by-field-of-view (model-names)
  (let* ((camera-pose (cl-transforms-stamped:lookup-transform
                       (ensure-tf-listener) "odom_combined" "head_tilt_link"
                       :timeout 2.0))
         (camera-fwd (cl-transforms-stamped:make-3d-vector 1 0 0))
         (camera-fwd (cl-transforms-stamped:rotate (cl-transforms-stamped:rotation camera-pose) camera-fwd))
         (camera-up (cl-transforms-stamped:make-3d-vector 0 0 1))
         (camera-up (cl-transforms-stamped:rotate (cl-transforms-stamped:rotation camera-pose) camera-up))
         (camera-pose (cl-transforms-stamped:translation camera-pose))
         (camera-pose (roslisp:make-message "geometry_msgs/Point"
                                            :x (cl-transforms-stamped:x camera-pose)
                                            :y (cl-transforms-stamped:y camera-pose)
                                            :z (cl-transforms-stamped:z camera-pose)))
         (camera-fwd (roslisp:make-message "geometry_msgs/Point"
                                           :x (cl-transforms-stamped:x camera-fwd)
                                           :y (cl-transforms-stamped:y camera-fwd)
                                           :z (cl-transforms-stamped:z camera-fwd)))
         (camera-up (roslisp:make-message "geometry_msgs/Point"
                                          :x (cl-transforms-stamped:x camera-up)
                                          :y (cl-transforms-stamped:y camera-up)
                                          :z (cl-transforms-stamped:z camera-up)))
         (focal-distance 1)
         (width 1)
         (height 1)
         (max-distance 12)
         (threshold 0.2))
    (cpl:mapcar-clean (lambda (model-name)
                        (roslisp:with-fields (visible)
                            (roslisp:call-service "/gazebo_visibility_ros/QueryGazeboVisibility"
                                                  "gazebo_visibility_ros/QueryGazeboVisibility"
                                                  :name model-name
                                                  :camera_pose camera-pose
                                                  :camera_fwd camera-fwd
                                                  :camera_up camera-up
                                                  :focal_distance focal-distance
                                                  :width width
                                                  :height height
                                                  :max_distance max-distance
                                                  :threshold threshold)
                         (unless (eql visible 0)
                           model-name)))
                      model-names)))

(defun find-objects (&key object-name)
  "Finds objects based on either their name `object-name' or their
type `object-type', depending what is given. An invalid combination of
both parameters will result in an empty list. When no parameters are
given, all known objects from the knowledge base are returned."
  (let* ((model-names (mapcar #'car (cram-gazebo-utilities:get-models)))
         (filtered-model-names (filter-models-by-ignored-objects
                                model-names))
         (filtered-model-names (filter-models-by-name
                                filtered-model-names
                                :template-name object-name))
         ;; TODO: Fix this external component; it returns all spawned
         ;; objects instead of the currently visible ones. This is
         ;; intended behavior and is related to problems in Gazebo
         ;; 2.2.3 w.r.t. raytracing code.
         ;(filtered-model-names (filter-models-by-field-of-view
         ;                       filtered-model-names))
         )
    (mapcar (lambda (model-name)
              (let ((pose (cram-gazebo-utilities:get-model-pose model-name)))
                (make-instance 'gazebo-designator-shape-data
                               :object-identifier model-name
                               :pose pose)))
            filtered-model-names)))

(defun find-with-designator (designator)
  (let* ((template-name (desig-prop-value designator :name))
         (template-type (desig-prop-value designator :type))
         (models (find-objects :object-name template-name)))
    (cpl:mapcar-clean (lambda (model)
                        (with-slots ((model-name desig::object-identifier) (pose desig::pose)) model
                          (let* ((pose (cram-gazebo-utilities:get-model-pose model-name))
                                 (location (make-designator :location `((:pose ,pose))))
                                 (description (cram-gazebo-utilities:spawned-object-description model-name))
                                 (description-type (cadr (find :type description :test (lambda (x y)
                                                                                         (eql x (car y))))))
                                 (model-data (make-instance 'gazebo-designator-shape-data
                                                            :object-identifier model-name
                                                            :pose pose)))
                            (when (or (and template-type (equal template-type description-type))
                                      (not template-type))
                              (make-effective-designator
                               designator
                               :new-properties (append `((:name ,model-name)
                                                         (:at ,location))
                                                       description)
                               :data-object model-data)))))
                      models)))

(defun get-bullet-objects ()
  (cpl:mapcar-clean
   #'identity
   (cut:force-ll
    (cut:lazy-mapcar
     (lambda (bdgs)
       (cut:with-vars-bound (?o) bdgs
         (when (stringp ?o) ?o)))
     (cram-prolog:prolog
      `(and (btr:bullet-world ?w)
            (btr:object ?w ?o)
            (not (btr::robot ?o))))))))

(defun update-bullet-object (name pose)
  (cram-prolog:prolog
   `(and (btr:bullet-world ?w)
         (btr:assert (btr:object ?w :box ,name ,pose)))))

(defun add-bullet-object (name pose dimensions)
  (cram-prolog:prolog
   `(and (btr:bullet-world ?w)
         ;;(btr:retract (btr:object ?w ,name))
         (btr:assert (btr:object ?w :box ,name ,pose
                                 :mass 0.1
                                 :size ,dimensions)))))

(defun update-belief-state (objects)
  (let* ((bullet-objects (get-bullet-objects))
         (new-objects
           (cpl:mapcar-clean (lambda (object)
                               (let* ((name (desig-prop-value object :name)))
                                 (unless (find name bullet-objects :test #'string=)
                                   object)))
                             objects))
         (present-objects
           (cpl:mapcar-clean (lambda (object)
                               (let ((name (desig-prop-value object :name)))
                                 (when (find name bullet-objects :test #'string=)
                                   object)))
                             objects)))
    (loop for object in present-objects do
      (let* ((at (desig-prop-value object :at))
             (pose (desig-prop-value at :pose))
             (name (desig-prop-value object :name)))
        (update-bullet-object name pose)))
    (loop for object in new-objects do
      (let* ((at (desig-prop-value object :at))
             (pose (desig-prop-value at :pose))
             (dimensions (desig-prop-value object :dimensions))
             (name (desig-prop-value object :name))
             (dimensions-list `(,(tf:x dimensions)
                                ,(tf:y dimensions)
                                ,(tf:z dimensions))))
        (add-bullet-object name pose dimensions-list)))))

(def-process-module gazebo-perception-process-module (input)
  (assert (typep input 'action-designator))
  (let* ((object-designator (desig-prop-value input :obj))
         (log-id (first (cram-language::on-prepare-perception-request object-designator))))
    (ros-info (gazebo perception-process-module) "Searching for object ~a" object-designator)
    (let ((results (find-with-designator object-designator)))
      (update-belief-state results)
      (cram-language::on-finish-perception-request log-id results)
      (if (not results)
          (cpl:fail 'cram-common-failures:perception-object-not-found :object-desig object-designator)
          results))))
