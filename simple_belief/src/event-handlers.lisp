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

(defmethod on-event attach-objects ((event object-attached))
  (format t "Attach object to gripper ~a.~%" (event-side event))
  (let* ((current-event-object (desig:current-desig (event-object event)))
         (attached-obj (find current-event-object *attached-objects*)))
    (cond ((eq attached-obj nil)
           (let*((target-frame (var-value '?target-frame
                                          (lazy-car
                                           (crs:prolog
                                            `(cram-pr2-knowledge::end-effector-link
                                              ,(event-side event)
                                              ?target-frame)))))
                 (obj-pose-in-gripper (tf:pose->pose-stamped
                                       target-frame
                                       0.0
                                       (cl-tf:transform-pose
                                        *tf*
                                        :pose (cram-designators:obj-desig-location
                                               (cram-designators:current-desig (event-object event)))
                                        :target-frame target-frame)))
                 (z-offset 0)
                 (loc-desig-in-gripper (cram-designators:make-designator
                                        'cram-designators:location
                                        (append `((pose ,obj-pose-in-gripper)
                                                  (gripper ,(event-side event))
                                                  (height ,z-offset)))))
                 (new-desig (update-grasped-object-designator current-event-object
                                                              loc-desig-in-gripper)))  
             (push (event-object event) *attached-objects*))
           ;;(desig:equate current-event-object new-desig))) 
           (let* ((r-target-frame (var-value '?target-frame
                                             (lazy-car
                                              (crs:prolog
                                               `(cram-pr2-knowledge::end-effector-link
                                                 ,:right
                                                 ?target-frame)))))
                  (obj-pose-in-r-gripper (tf:pose->pose-stamped
                                          r-target-frame
                                          0.0
                                          (cl-tf:transform-pose
                                           *tf*
                                           :pose (cram-designators:obj-desig-location
                                                  (cram-designators:current-desig (event-object event)))
                                             :target-frame r-target-frame)))
                  (l-target-frame (var-value '?target-frame
                                             (lazy-car
                                              (crs:prolog
                                               `(cram-pr2-knowledge::end-effector-link
                                                 ,:left
                                                 ?target-frame)))))
                  (obj-pose-in-l-gripper (tf:pose->pose-stamped
                                          l-target-frame
                                          0.0
                                          (cl-tf:transform-pose
                                           *tf*
                                           :pose (cram-designators:obj-desig-location
                                                  (cram-designators:current-desig (event-object event)))
                                           :target-frame l-target-frame)))
                  (z-offset 0)
                  (loc-desig-in-gripper (cram-designators:make-designator
                                         'cram-designators:location
                                         (append `((pose ,obj-pose-in-r-gripper)
                                                   (gripper ,:right)
                                                   (pose, obj-pose-in-l-gripper)
                                                   (gripper ,:left)
                                                   (height ,z-offset))))))
             (update-grasped-object-designator current-event-object
                                               loc-desig-in-gripper)
       (push (event-object event) *attached-objects*))))))

(defmethod on-event detach-objects ((event object-detached))
  (format t "Detach object from gripper ~a.~%" (event-side event))
  (setf *attached-objects* 
        (remove (event-object event) *attached-objects*
                :test (lambda (a b)
                        (equal (car a)(car b))
                        (equal (cdr a)(cdr b))))))
