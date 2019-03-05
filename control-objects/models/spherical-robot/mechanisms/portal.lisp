;;;; portal.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defparameter *sphere/portal/number-of-legs* 3)
(defparameter *sphere/portal/legs-actuators-types* #(p p p))
(defparameter *sphere/portal/endeffector-relative-radius* 0.2d0)
(defparameter *sphere/portal/legs-setpoints-calculation-function*
  (lambda (endeffector-position &optional endeffector-rotation)
    (declare (ignore endeffector-rotation))
    endeffector-position))
(defparameter *sphere/portal/max-endeffector-relative-displacement* 0.5d0)

;;; -------------------------------------------------------------------------

(defun sphere/install-portal-mechanism ()
  (setf *sphere/number-of-legs* *sphere/portal/number-of-legs*)
  (setf *sphere/legs-actuators-types* *sphere/portal/legs-actuators-types*)
  (setf *sphere/endeffector-relative-radius*
     *sphere/portal/endeffector-relative-radius*)
  (setf *sphere/legs-setpoints-calculation-function*
     *sphere/portal/legs-setpoints-calculation-function*)
  (setf *sphere/max-endeffector-relative-displacement*
     *sphere/portal/max-endeffector-relative-displacement*)
  (setf *sphere/mechanism-type* "portal"))
