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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors
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

(in-package :gazebo-perception-pm)

(defun designator-model-pose (name)
  (let ((found-objects (find-object :object-name name)))
    (when found-objects
      (object-pose (first found-objects)))))

;; NOTE(winkler): When objects are referred to by their type rather
;; than their name, multiple instances can show up in the
;; results. When requested to look at these objects (i.e. a location
;; to look at this object group), the pose of the first object (if
;; any) is returned. This results in the robot always looking towards
;; the first object that is returned by perception.
(defun designator-model-pose-from-type (type)
  (let ((found-objects (find-object :object-type type)))
    (when found-objects
      (object-pose (first found-objects)))))

(def-fact-group process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator gazebo-perception-process-module)
    (desig-prop ?designator (to perceive))
    (desig-prop ?designator (obj ?object))
    (obj-desig? ?object))

  (<- (available-process-module gazebo-perception-process-module)
    (symbol-value cram-projection:*projection-environment* nil)))

(def-fact-group perception-action-designator (action-desig)

  (<- (action-desig ?desig ?object)
    (desig-prop ?desig (to perceive))
    (desig-prop ?desig (obj ?object))))

(def-fact-group gazebo-object-locations (desig-solution)

  (<- (desig-solution ?designator ?solution)
    (desig-prop ?designator (of ?object))
    (obj-desig? ?object)
    (desig-prop ?object (name ?name))
    (lisp-fun designator-model-pose ?name ?solution)
    (lisp-pred identity ?solution))

  (<- (desig-solution ?designator ?solution)
    (desig-prop ?designator (of ?object))
    (obj-desig? ?object)
    (desig-prop ?object (type ?type))
    (lisp-fun designator-model-pose-from-type ?type ?solution)
    (lisp-pred identity ?solution)))
