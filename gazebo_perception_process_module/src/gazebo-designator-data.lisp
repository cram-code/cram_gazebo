;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(defparameter *urdf-cache* (make-hash-table :test #'equal)
  "Cache for storing loaded URDF files.")

(defclass gazebo-designator-data (object-designator-data)
  ((type :initarg :type :reader object-type)))

(defclass gazebo-designator-mesh-data
    (gazebo-designator-data cram-manipulation-knowledge:object-mesh-data-mixin)
  ())

(defclass gazebo-designator-shape-data
    (gazebo-designator-data cram-manipulation-knowledge:object-shape-data-mixin)
  ())

(defun get-object-geometry (urdf &key (mesh-identifier :visual))
  "Returns an instance of CL-URDF:GEOMETRY for the root link in
`urdf'. `urdf' can be a pathname, a string containing the URDF or a
stream. `mesh-identifier' can be used to select either the visual
element (:VISUAL) or the collision element (:COLLISION)."
  (declare (type (or pathname string stream) urdf))
  (let ((parsed-urdf (or (gethash urdf *urdf-cache*)
                         (setf (gethash urdf *urdf-cache*)
                               (cl-urdf:parse-urdf urdf)))))
    (assert
     (eql (hash-table-count (cl-urdf:links parsed-urdf)) 1) ()
     "At the moment, only object URDF files with exactly one link are supported.")
    (cl-urdf:geometry
     (ecase mesh-identifier
       (:visual (cl-urdf:visual (cl-urdf:root-link parsed-urdf)))
       (:collision (cl-urdf:collision (cl-urdf:root-link parsed-urdf)))))))

(defun get-object-geometry-pose (urdf &key (mesh-identifier :visual))
  "Returns a CL-TRANSFORMS:POSE for the geometry element indicated by
  `mesh-identified' of root link in `urdf'"
  (let ((parsed-urdf (or (gethash urdf *urdf-cache*)
                         (setf (gethash urdf *urdf-cache*)
                               (cl-urdf:parse-urdf urdf)))))
    (assert
     (eql (hash-table-count (cl-urdf:links parsed-urdf)) 1) ()
     "At the moment, only object URDF files with exactly one link are supported.")
    (cl-urdf:origin
     (ecase mesh-identifier
       (:visual (cl-urdf:visual (cl-urdf:root-link parsed-urdf)))
       (:collision (cl-urdf:collision (cl-urdf:root-link parsed-urdf)))))))

(defun 3d-model-faces->indices (3d-model)
  "Returns the faces of `3d-model' as sequences of indices in the
  vertex field instead of sequences of points."
  (declare (type physics-utils:3d-model 3d-model))
  (flet ((point-index (vertex vertices)
           (position vertex vertices
                     :test (lambda (a b)
                             (< (cl-transforms:v-dist a b) 1e-6)))))
    (let ((vertices (physics-utils:3d-model-vertices 3d-model))
          (faces (physics-utils:3d-model-faces 3d-model)))
      (map 'vector (lambda (face)
                     (mapcar (lambda (point)
                               (point-index point vertices))
                             (physics-utils:face-points face)))
           faces))))

(defgeneric geometry->designator-data (name pose type geometry
                                       &optional geometry-pose)
  (:documentation "Returns an instance of a subclass of
  OBJECT-DESIGNATOR-DATA matching `geometry'.")

  (:method (name pose type (mesh cl-urdf:mesh)
            &optional (geometry-pose (cl-transforms:make-identity-pose)))
    (let ((mesh (physics-utils:transform-3d-model
                 (cl-urdf:3d-model mesh) geometry-pose)))
      (make-instance 'gazebo-designator-mesh-data
        :object-identifier name
        :type type
        :pose pose
        :vertices (physics-utils:3d-model-vertices mesh)
        :faces (3d-model-faces->indices mesh))))
  
  (:method (name pose type (box cl-urdf:box)
            &optional (geometry-pose (cl-transforms:make-identity-pose)))
    (make-instance 'gazebo-designator-shape-data
      :object-identifier name
      :type type
      :pose (cl-transforms:transform-pose
             (cl-transforms:pose->transform pose) geometry-pose)
      :shape-type :box
      :dimensions (cl-urdf:size box)))
  
  (:method (name pose type (cylinder cl-urdf:cylinder)
            &optional (geometry-pose (cl-transforms:make-identity-pose)))
    (make-instance 'gazebo-designator-shape-data
      :object-identifier name
      :type type
      :pose (cl-transforms:transform-pose
             (cl-transforms:pose->transform pose) geometry-pose)
      :shape-type :cylinder
      :dimensions (cl-transforms:make-3d-vector
                   (cl-urdf:length cylinder)
                   (cl-urdf:radius cylinder)
                   (cl-urdf:radius cylinder))))

  (:method (name pose type (sphere cl-urdf:sphere)
            &optional (geometry-pose (cl-transforms:make-identity-pose)))
    (make-instance 'gazebo-designator-shape-data
      :object-identifier name
      :type type
      :pose (cl-transforms:transform-pose
             (cl-transforms:pose->transform pose) geometry-pose)
      :shape-type :sphere
      :dimensions (cl-transforms:make-3d-vector
                   (cl-urdf:radius sphere)
                   (cl-urdf:radius sphere)
                   (cl-urdf:radius sphere)))))
