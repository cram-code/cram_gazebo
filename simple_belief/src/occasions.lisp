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
      (prolog `(holds ?_  ,occasion ,time-specification))
      (prolog `(holds ,occasion))))

(defun is-robot-at-location (temp-loc)
  (let* ((robot-pose (gazebo-perception-pm::get-model-pose "pr2"))
	 (temp-pose (desig:reference temp-loc)))
    (are-poses-similar temp-pose robot-pose)))

(defun are-poses-similar (pose1 pose2)
  (let ((abs-offset-max 0.01)) ; Threshold 0.01
    (if (< (abs (- (tf:x (tf:origin pose1)) (tf:x (tf:origin pose2)))) abs-offset-max)
	(if (< (abs (- (tf:y (tf:origin pose1)) (tf:y (tf:origin pose2)))) abs-offset-max)
	    (if (< (abs (- (tf:z (tf:origin pose1)) (tf:z (tf:origin pose2)))) abs-offset-max)
		T)))
    nil))

(def-fact-group occasions (holds)

  (<- (object-in-hand ?object ?side)
    (symbol-value *attached-objects* ?attached-objects)
    (member (?object . ?side) ?attached-objects))
  
  (<- (object-in-hand ?object)
    (object-in-hand ?object ?_))

  (<- (loc plan-knowledge:robot ?location)
    ;; Why does this not work with only one argument?
    (lisp-fun is-robot-at-location ?location ?location))

  (<- (loc ?object ?location)
    ;; This could fail due to mechanisms being different here than in
    ;; the original cram_environment_representation.
    (desig:obj-desig? object)
    (object-designator-name ?object ?object-name)
    (object-at-location ?_ ?object-name ?location))

  (<- (holds ?occasion)
    (call ?occasion)))

(def-fact-group occasion-utilities ())