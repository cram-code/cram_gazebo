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

(defsystem cram-gazebo-utilities
  :author "Jan Winkler <winkler@cs.uni-bremen.de>"
  :license "BSD"
  :description "CRAM gazebo utilities"

  :depends-on (cram-roslisp-common
               cram-language
               cram-reasoning
               process-modules
               cram-utilities
               cram-plan-knowledge
               designators
               designators-ros
               actionlib
               semantic-map-cache
               ;; vision_msgs-msg
               ;; vision_srvs-srv
               std_msgs-msg
               pr2_msgs-msg
	       trajectory_msgs-msg
               cram-plan-failures
               gazebo_msgs-msg
               gazebo_msgs-srv
               cl-semantic-map-utils
               cram-plan-library
	       attache-msgs)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "ros" :depends-on ("package"))
     (:file "occasions" :depends-on ("ros" "package"))
     (:file "utils" :depends-on ("package" "ros" "occasions"))))))
