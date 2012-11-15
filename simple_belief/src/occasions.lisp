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

(in-package :simple-belief)

(defmethod cram-plan-knowledge:holds (occasion &optional time-specification)
  (if time-specification
      (prolog `(holds ?_ ,occasion ,time-specification))
      (prolog `(holds ,occasion))))

(defun is-robot-at-location (temp-loc)
  (let* ((robot-pose (cram-gazebo-utilities:get-model-pose "pr2"))
	 (temp-pose (desig:reference temp-loc)))
    (poses-equal-p temp-pose robot-pose)))

(defun poses-equal-p (pose-1 pose-2 &key
			     (position-threshold 0.01)
			     (angle-threshold (/ pi 180)))
  (declare (type cl-transforms:pose pose-1 pose-2))
  (and (< (tf:v-dist (tf:origin pose-1) (tf:origin pose-2)) position-threshold)
       (< (tf:angle-between-quaternions
	   (tf:orientation pose-1)
	   (tf:orientation pose-2))
	  angle-threshold)))

;; NOTE(winkler): This function has to be implemented. Using this, the
;; predicate `looking-at' will determine whether the robot's PTU is
;; actually facing the point given as `loc'.
(defun robot-looking-at (loc)
  (let ((target-pose-stamped (desig:reference loc))
	(head-pointing-at-pose-stamped nil))
    ;; NOTE(winkler): Returning `nil' always, so that the PTU will be
    ;; panned and tilted towards the target pose `target-pose-stamped'
    ;; in any case. The current pointing angles of the PTU have to be
    ;; read, though.
    ))

(def-fact-group occasions (holds)

  (<- (object-in-hand ?object)
    (object-in-hand ?object ?_))

  (<- (object-in-hand ?object ?side)
    (symbol-value *attached-objects* ?attached-objects)
    (bagof (?obj-transf . ?side) (transformed-object-desig ?side
                                                           ?attached-objects
                                                           ?obj-transf)
           ?att-objs)
    (member (?object . ?side) ?att-objs))

  (<- (transformed-object-desig ?side ?attached-objects ?obj-transf)
    (member (?obj . ?side) ?attached-objects)
    (lisp-fun desig:current-desig ?obj ?obj-transf))

  (<- (loc plan-knowledge:robot ?location)
    (lisp-pred is-robot-at-location ?location))

  (<- (loc ?object ?location)
    (desig:obj-desig? ?object)
    (desig:obj-desig? ?location)
    (lisp-fun desig:designator-pose ?object ?object-pose)
    (lisp-fun desig:reference ?location ?pose)
    (lisp-pred poses-equal-p ?object-pose ?pose))

  (<- (loc ?object ?location)
    (not (bound ?location))
    (desig:obj-desig? ?object)
    (desig:designator 'desig:location ((desig-props:of ?object)) ?location))

  ;; NOTE(winkler): This has to be rewritten into a calculation of
  ;; whether the robot is looking into the direction of the location
  ;; or not. This should be doable with a simple line calculation,
  ;; originating from the head frame, ending in `?location'. The
  ;; orientation vector then should be compared to the current head
  ;; orientation.
  (<- (looking-at ?location)
    (lisp-pred robot-looking-at ?location))

  (<- (holds ?occasion)
    (call ?occasion)))
