;;;; reference-frame.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

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
