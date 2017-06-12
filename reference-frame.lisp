;;;; reference-frame.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defun multiply-matrix3x3-by-vector3 (mat vec)
  (vector
   (+ (* (aref mat 0 0) (cartesian/x vec))
      (* (aref mat 0 1) (cartesian/y vec))
      (* (aref mat 0 2) (cartesian/z vec)))
   
   (+ (* (aref mat 1 0) (cartesian/x vec))
      (* (aref mat 1 1) (cartesian/y vec))
      (* (aref mat 1 2) (cartesian/z vec)))
   
   (+ (* (aref mat 2 0) (cartesian/x vec))
      (* (aref mat 2 1) (cartesian/y vec))
      (* (aref mat 2 2) (cartesian/z vec)))))

(defun invert-rotation-matrix (rot-matrix)
  (when rot-matrix
    (let ((inv-rot-matrix (make-array '(3 3) :element-type 'double-float)))
      (setf (aref inv-rot-matrix 0 0) (aref rot-matrix 0 0)
         (aref inv-rot-matrix 0 1) (aref rot-matrix 1 0)
         (aref inv-rot-matrix 0 2) (aref rot-matrix 2 0)
         (aref inv-rot-matrix 1 0) (aref rot-matrix 0 0)
         (aref inv-rot-matrix 1 1) (aref rot-matrix 1 1)
         (aref inv-rot-matrix 1 2) (aref rot-matrix 2 1)
         (aref inv-rot-matrix 2 0) (aref rot-matrix 0 2)
         (aref inv-rot-matrix 2 1) (aref rot-matrix 1 2)
         (aref inv-rot-matrix 2 2) (aref rot-matrix 2 2))
      inv-rot-matrix)))

(defun rotate-coordinates-by-matrix (rot-matrix coord)
  (if (null rot-matrix)
      coord
      (let* ((vec (coordinates/get-vector coord))
             (rotated (coordinates
                       :v (multiply-matrix3x3-by-vector3 rot-matrix vec))))
        (coordinates/set-coordinate-system
         rotated (coordinates/get-coordinate-system coord))
        rotated)))

;;; -------------------------------------------------------------------------

(defun point (coordinates reference-frame/world-p)
  (cons coordinates reference-frame/world-p))

(defmacro point/coordinates (p)
  `(car ,p))

(defmacro point/reference-frame-world-p (p)
  `(cdr ,p))

(defun transform-reference-frame (pnt target-rf/world-p local-rf/rot-matrix
                                  &optional (local-rf/inv-rot-matrix
                                             (invert-rotation-matrix
                                              local-rf/rot-matrix)))
  (if (eql (point/reference-frame-world-p pnt) target-rf/world-p)
      pnt
      (point (rotate-coordinates-by-matrix
              (if (not target-rf/world-p)
                  local-rf/rot-matrix local-rf/inv-rot-matrix)
              (point/coordinates pnt)) target-rf/world-p)))
