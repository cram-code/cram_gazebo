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

(defvar *attached-objects* nil)

(defmethod on-event attach-objects ((event object-attached))
  (format t "Attach ~a to gripper ~a.~%" (event-object event) (event-side event))
  (update-grasped-object-designator (event-object event) (list (event-side event)))
  (push (cons (event-object event) (event-side event)) *attached-objects*))

(defmethod on-event detach-objects ((event object-detached))
  (format t "Detach.~%"))

(defun update-grasped-object-designator (obj grippers &key new-properties)
  (let* ((target-frame (var-value '?target-frame
                                  (lazy-car
                                   (crs:prolog
                                    `(cram-pr2-knowledge::end-effector-link
                                      ,(car grippers)
                                      ?target-frame)))))
         (obj-pose-in-gripper (tf:pose->pose-stamped
                               target-frame
                               0.0
                               (cl-tf:transform-pose
                                *tf*
                                :pose (cram-designators:obj-desig-location
                                       (cram-designators:current-desig obj))
                                :target-frame target-frame)))
         (loc-desig-in-gripper (cram-designators:make-designator
                                'cram-designators:location
                                (append `((pose ,obj-pose-in-gripper)
                                          (in gripper))
                                        (mapcar (lambda (grip)
                                                  `(gripper ,grip))
                                                grippers)))))
    (cram-designators:make-designator
     'object
     (append `((at ,loc-desig-in-gripper) .
               ,(remove 'at (cram-designators:description obj)
                        :key #'car))
             new-properties)
     obj)))
