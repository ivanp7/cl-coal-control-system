;;;; 6SUR.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defparameter *sphere/6SUR/RU-chain-relative-length* 0.8d0)
(defparameter *sphere/6SUR/US-chain-relative-length* 1.0d0)

(defparameter *sphere/6SUR/endeffector-relative-radius* 0.25d0)

;;; -------------------------------------------------------------------------

(defparameter *sphere/6SUR/number-of-legs* 6)
(defparameter *sphere/6SUR/legs-actuators-types* #(r r r r r r))

(defparameter *sphere/6SUR/max-endeffector-relative-displacement* 0.2d0)

;;; -------------------------------------------------------------------------

(defparameter *sphere/6SUR/R-angle-initial-displacement* -0.1939495730766756d0)

;;; -------------------------------------------------------------------------

(defparameter *sphere/6SUR/end-effector-rotation-inv-matrix*
  (invert-rotation-matrix
   (convert-quaternion-to-matrix3x3
    (let ((alpha (* pi 7/24)))
      (let ((e (/ (sin (/ alpha 2)) (sqrt 3))))
        (vector (cos (/ alpha 2)) e e e))))))

(defparameter *sphere/6SUR/leg-cs-to-end-effector-cs-rotation-matrices*
  (mapcar #'convert-quaternion-to-matrix3x3
          (list #(1.0d0 0.0d0 0.0d0 0.0d0) #(1.0d0 0.0d0 0.0d0 0.0d0)
                #(0.5d0 0.5d0 0.5d0 0.5d0) #(0.5d0 0.5d0 0.5d0 0.5d0)
                #(0.5d0 -0.5d0 -0.5d0 -0.5d0) #(0.5d0 -0.5d0 -0.5d0 -0.5d0))))

(defparameter *sphere/6SUR/end-effector-cs-reflection-coefficients*
  (list #(-1.0d0 1.0d0 1.0d0) #(1.0d0 -1.0d0 -1.0d0)
        #(1.0d0 -1.0d0 1.0d0) #(-1.0d0 1.0d0 -1.0d0)
        #(1.0d0 1.0d0 -1.0d0) #(-1.0d0 -1.0d0 1.0d0)))

(defparameter *sphere/6SUR/end-effector-cs-to-leg-cs-rotation-matrices*
  (mapcar #'invert-rotation-matrix
          *sphere/6SUR/leg-cs-to-end-effector-cs-rotation-matrices*))

(defun sphere/6SUR/calculate-leg-ends-coordinates (endeffector-position)
  (mapcar (lambda (leg-cs-to-endeff-cs-rotation-mat
              leg-cs-to-endeff-cs-reflection-coef
              endeff-cs-to-leg-cs-rotation-mat)
            (multiply-matrix3x3-by-vector3
             endeff-cs-to-leg-cs-rotation-mat
             (v*e (multiply-matrix3x3-by-vector3
                   *sphere/6SUR/end-effector-rotation-inv-matrix*
                   (v- (v*e (multiply-matrix3x3-by-vector3
                             leg-cs-to-endeff-cs-rotation-mat
                             (vector 0.0d0 0.0d0 *sphere/radius*))
                            leg-cs-to-endeff-cs-reflection-coef)
                       endeffector-position))
                  leg-cs-to-endeff-cs-reflection-coef)))
          *sphere/6SUR/leg-cs-to-end-effector-cs-rotation-matrices*
          *sphere/6SUR/end-effector-cs-reflection-coefficients*
          *sphere/6SUR/end-effector-cs-to-leg-cs-rotation-matrices*))

(defun sphere/6SUR/calculate-R-angle (ev)
  (let ((ex (elt ev 0)) (ey (elt ev 1)) (ez (elt ev 2))
        (endeff-radius (* *sphere/radius* *sphere/endeffector-relative-radius*))
        (ru-length (* *sphere/radius* *sphere/6SUR/RU-chain-relative-length*))
        (us-length (* *sphere/radius* *sphere/6SUR/US-chain-relative-length*)))
    (flet ((^2 (value) (* value value)))
      (let ((a ru-length)
            (b (* us-length
                  (- (sqrt (- 1.0d0 (^2 (/ (- ez endeff-radius)
                                       us-length)))))))
            (c1 ex) (c2 ey))
        
        (let ((a^2 (^2 a)) (b^2 (^2 b)) (c1^2 (^2 c1)) (c2^2 (^2 c2)))
          (- (if (< (abs c2) 1.0d-3)
                 (atan (- (/ (sqrt (- (+ (^2 a^2)
                                      (^2 (- b^2 c1^2))
                                      (* -2 a^2 (+ b^2 c1^2)))))
                             c1))
                       (/ (+ a^2 (- b^2) c1^2)
                          c1))
                 (let* ((c1^2+c2^2 (+ c1^2 c2^2))
                        (a^2-b^2+c1^2+c2^2 (+ a^2 (- b^2) c1^2+c2^2))
                        (radix (sqrt (* (- c2^2)
                                     (+ (^2 a^2)
                                        (^2 (- c1^2+c2^2 b^2))
                                        (* -2 a^2 (+ c1^2+c2^2 b^2)))))))
                   (atan (/ (- (* c2^2 a^2-b^2+c1^2+c2^2) (* c1 radix))
                            (* c2 c1^2+c2^2))
                         (/ (+ (* c1 a^2-b^2+c1^2+c2^2) radix)
                            c1^2+c2^2))))
             *sphere/6SUR/R-angle-initial-displacement*))))))

(defparameter *sphere/6SUR/legs-setpoints-calculation-function*
  (lambda (endeffector-position &optional endeffector-rotation)
    (declare (ignore endeffector-rotation))
    (mapcar #'sphere/6SUR/calculate-R-angle
            (sphere/6SUR/calculate-leg-ends-coordinates endeffector-position))))

;;; -------------------------------------------------------------------------

(defun sphere/install-6SUR-mechanism ()
  (setf *sphere/number-of-legs* *sphere/6SUR/number-of-legs*)
  (setf *sphere/legs-actuators-types* *sphere/6SUR/legs-actuators-types*)
  (setf *sphere/endeffector-relative-radius*
     *sphere/6SUR/endeffector-relative-radius*)
  (setf *sphere/legs-setpoints-calculation-function*
     *sphere/6SUR/legs-setpoints-calculation-function*)
  (setf *sphere/max-endeffector-relative-displacement*
     *sphere/6SUR/max-endeffector-relative-displacement*)
  (setf *sphere/mechanism-type* "6-SU[R]"))
