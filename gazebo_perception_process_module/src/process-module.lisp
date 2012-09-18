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

(defvar *gazebo-modelstates-subscriber* nil)
(defvar *model-state-msg* (cram-language:make-fluent :name :model-state-msg))
(defvar *perception-role* nil)

(defparameter *known-roles* '(gazebo-detector)
  "Ordered list of known roles for designator resolution. They are
  processed in the order specified in this list")

(defmacro def-object-search-function (function-name role (props desig perceived-object)
                                      &body body)
  (check-type function-name symbol)
  (check-type role symbol)
  (check-type props list)
  (check-type desig symbol)
  (check-type perceived-object symbol)
  (assert (every #'listp props) ()
          "The parameter `props' is not a valid designator property list")
  `(progn
     (defun ,function-name (,desig ,perceived-object)
       ,@body)

     (def-fact-group ,(intern (concatenate 'string (symbol-name function-name) "-FACTS"))
         (object-search-function object-search-function-order)
       
       (<- (object-search-function ?desig ,role ?fun)
         ,@(mapcar (lambda (prop)
                     `(desig-prop ?desig ,prop))
                   props)
         (lisp-fun symbol-function ,function-name ?fun))

       (<- (object-search-function-order ?fun ,(length props))
         (lisp-fun symbol-function ,function-name ?fun)))))

(defgeneric make-new-desig-description (old-desig perceived-object)
  (:documentation "Merges the description of `old-desig' with the
properties of `perceived-object'")
  (:method ((old-desig object-designator) (po object-designator-data))
    (let ((obj-loc-desig (make-designator 'location `((pose ,(object-pose po))))))
      (cons `(at ,obj-loc-desig)
            (remove 'at (description old-desig) :key #'car)))))

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
           (pose-sequence pose))
;           (twist-sequence twist))
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

(def-process-module gazebo-perception-process-module (input)
  (assert (typep input 'action-designator))
  (let ((object-designator (reference input)))
    (ros-info (gazebo-perception-pm process-module) "Searching for object ~a" object-designator)
    (let* ((newest-effective (newest-effective-designator object-designator))
           (result
             (some (lambda (role)
                     (let ((*perception-role* role))
                       (if newest-effective
                           ;; Designator that has alrady been equated
                           ;; to one with bound to a perceived-object
                           (find-with-parent-desig newest-effective)
                           (find-with-new-desig object-designator))))
                   *known-roles*)))
      (unless result
        (cram-language:fail 'object-not-found :object-desig object-designator))
      (ros-info (gazebo-perception-pm process-module) "Found objects: ~a" result)
      result)))

(defun make-handled-object-designator (&key object-type
                                            object-pose
                                            handles
                                            name)
  "Creates and returns an object designator with object type
`object-type' and object pose `object-pose' and attaches location
designators according to handle information in `handles'."
  (let ((combined-description (append `((type ,object-type)
                                        (name ,name)
                                        (at ,(make-designator
                                              'location `((pose ,object-pose)))))
                                      `,(make-handle-designator-sequence handles))))
    (make-designator 'object combined-description)))

(defun make-handle-designator-sequence (handles)
  "Converts the sequence `handles' (handle-pose handle-radius) into a
sequence of object designators representing handle objects. Each
handle object then consist of a location designator describing its
relative position as well as the handle's radius for grasping
purposes."
  (mapcar (lambda (handle-desc)
            `(handle
              ,(make-designator 'object
                                `((at ,(make-designator 'location `((pose ,(first handle-desc)))))
                                  (radius ,(second handle-desc))
                                  (type handle)))))
          handles))

(defclass perceived-object (object-designator-data)
  ((designator :reader object-designator :initarg :designator)))

(defmethod designator-pose ((designator object-designator))
  (object-pose (reference designator)))

(defmethod designator-distance ((designator-1 object-designator)
                                      (designator-2 object-designator))
  (cl-transforms:v-dist (cl-transforms:origin (designator-pose designator-1))
                        (cl-transforms:origin (designator-pose designator-2))))

(defun make-object-designator (perceived-object &key parent type name)
  (assert parent)
  (let ((pose (object-pose perceived-object)))
    (make-effective-designator
     parent
     :new-properties (update-designator-properties
                      `(,@(when type `((type ,type)))
                        (at ,(make-designator 'location `((pose ,pose))))
                        ,@(when name `((name ,name))))
                      (when parent (properties parent)))
     :data-object perceived-object)))

(defun find-object-with-id (id &key name type)
  (find-object (make-named-object-designator id :name name :type type)))

(defun make-named-object-designator (id &key name type)
  (declare (ignore name))
  (let ((obj-desig (gazebo-perception-process-module::make-object-designator
                    (make-instance 'gazebo-perception-pm::perceived-object
                      :object-identifier id
                      :pose (get-model-pose id))
                    :parent (cram-designators::make-designator 'cram-designators::object ())
                    :name id;name
                    :type type)))
    obj-desig))

(defun find-object (designator)
  "Finds objects with (optional) name `object-name' and type `type'
and returns a list of elements of the form \(name pose\)."
  (let ((object-name (when (slot-value designator 'data)
                       (object-identifier (reference designator)))))
    (list (list object-name (get-model-pose object-name)))))

;; (defun find-with-bound-designator (designator)
;;   (flet ((make-designator (object pose)
;;            (make-object-designator
;;             (make-instance
;;              'perceived-object
;;              :object-identifier object
;;              :pose pose)
;;             :name object
;;             :parent designator)))
;;     (cut:force-ll
;;      (cut:lazy-mapcar
;;       (alexandria:curry #'apply #'make-designator) (find-object designator)))))
(defun find-with-parent-desig (desig)
  "Takes the perceived-object of the parent designator as a bias for
   perception."
  (let* ((parent-desig (current-desig desig))
         (perceived-object (reference (newest-effective-designator parent-desig))))
    (or
     (when perceived-object
       (let ((perceived-objects
               (execute-object-search-functions parent-desig :perceived-object perceived-object)))
         (when perceived-objects
           ;; NOTE(winkler): Removed the (car ...) here due to the
           ;; fact that find-with-new-desig returns a list. This
           ;; function should return a list as well, because otherwise
           ;; `gazebo-perception-process-module (input)` returns
           ;; either a list or a single element. They should return
           ;; the same data type.  (car ...
           (mapcar (lambda (perceived-object)
                          (emit-perception-event
                           (perceived-object->designator parent-desig perceived-object)))
                        perceived-objects))))
     (find-with-new-desig desig))))

(defun find-with-new-desig (desig)
  "Takes a parent-less designator. A search is performed a new
   designator is generated for every object that has been found."
  (let ((perceived-objects (execute-object-search-functions desig :role *perception-role*)))
    ;; Sort perceived objects according to probability
    (mapcar (lambda (perceived-object)
              (emit-perception-event
               (perceived-object->designator desig perceived-object)))
            perceived-objects)))

(defun emit-perception-event (designator)
  (cram-plan-knowledge:on-event (make-instance 'cram-plan-knowledge:object-perceived-event
                                  :perception-source :gazebo-perception-process-module
                                  :object-designator designator))
  designator)

(defclass handle-perceived-object (object-designator-data) ())

(defmethod make-new-desig-description ((old-desig object-designator)
                                       (perceived-object perceived-object))
  (let ((description (call-next-method)))
    (if (member 'name description :key #'car)
        description
        (cons `(name ,(object-identifier perceived-object)) description))))

(defun perceived-object->designator (desig obj)
  (make-effective-designator
   desig :new-properties (make-new-desig-description desig obj)
         :data-object obj))

(defun execute-object-search-functions (desig &key perceived-object (role *perception-role*))
  "Executes the matching search functions that fit the properties of
   `desig' until one succeeds. `role' specifies the role under which
   the search function should be found. If `role' is set to NIL, all
   matching search functins are used. The order in which the search
   functions are executed is determined by the number of designator
   properties that are matched. Functions that are more specific,
   i.e. match more pros are executed first. `perceived-object' is an
   optional instance that previously matched the object."
  (let ((obj-search-functions (force-ll
                               (lazy-mapcar
                                (lambda (bdg)
                                  (with-vars-bound (?role ?fun ?order) bdg
                                    (list ?fun ?role ?order)))
                                (prolog `(and (object-search-function ,desig ?role ?fun)
                                              (object-search-function-order ?fun ?order))
                                        (when role
                                          (add-bdg '?role role nil)))))))
    (some (lambda (fun) (funcall (first fun) desig perceived-object))
          (sort obj-search-functions #'> :key #'third))))

(def-object-search-function gazebo-object-search-function gazebo-detector
    (() desig perceived-object)
  (declare (ignore perceived-object))
  (let* ((pose (reference (desig-prop-value desig 'at)))
         (pose-transformed (tf:make-pose-stamped (tf:frame-id pose) 0.0 (tf:origin pose) (tf:orientation pose))))
    (list (make-instance 'perceived-object
                         :object-identifier (desig-prop-value desig 'name)
                         :pose pose-transformed))))

(cram-roslisp-common:register-ros-init-function init-gazebo-perception-process-module)