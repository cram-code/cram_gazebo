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

(in-package :simple-knowledge)

(def-fact-group utilities ()
  (<- (equate-object-type-symbols ?lhs ?rhs)
    (bound ?lhs)
    (bound ?rhs)
    (lisp-fun symbol-name ?lhs ?lhs-name)
    (lisp-fun symbol-name ?rhs ?rhs-name)
    (lisp-pred equal ?lhs-name ?rhs-name))

  (<- (equate-object-type-symbols ?symbol ?symbol)))

(def-fact-group gazebo-object-types ()

  (<- (gazebo-object ?object ?name ?type)
    (symbol-value *object-list* ?objects)
    (member ?object ?objects)
    (get-slot-value ?object object-name ?obj-name)
    (equal ?name ?obj-name)
    (get-slot-value ?object object-type ?object-type)
    (equate-object-type-symbols ?object-type ?type))

  (<- (object-handles ?name ?handles)
    (gazebo-object ?object ?name ?_)
    (get-slot-value ?object handles ?handles))

  (<- (object-collision-parts ?name ?collision-parts)
    (gazebo-object ?object ?name ?_)
    (get-slot-value ?object collision-parts ?collision-parts))

  (<- (object-min-handles ?name ?min-handles)
    (gazebo-object ?object ?name ?_)
    (get-slot-value ?object min-handles ?min-handles))

  (<- (object-height ?name ?height)
    (gazebo-object ?object ?name ?_)
    (get-slot-value ?object height ?height))

  (<- (object-filename ?name ?filename)
    (gazebo-object ?object ?name ?_)
    (get-slot-value ?object filename ?filename)))

