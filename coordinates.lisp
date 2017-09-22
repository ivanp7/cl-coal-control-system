;;;; coordinates.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(alexandria:define-constant +null-vector+ #(0.0d0 0.0d0 0.0d0) :test #'equalp)

;;; -------------------------------------------------------------------------

(defmacro cartesian/x (v)
  `(elt ,v 0))

(defmacro cartesian/y (v)
  `(elt ,v 1))

(defmacro cartesian/z (v)
  `(elt ,v 2))

(defmacro spherical/rho (v)
  `(elt ,v 0))

(defmacro spherical/phi (v)
  `(elt ,v 1))

(defmacro spherical/theta (v)
  `(elt ,v 2))

;;; -------------------------------------------------------------------------

;; plist of (cons <from cartesian c.s. conv. fn.> <to cartesian c.s. conv. fn.>)
(defparameter *coord-system-conversion-fn-plist*
  (list :spherical
        (cons (lambda (vec) ; from cartesian c.s.
                (flet ((^2 (a) (* a a)))
                  (let ((x (cartesian/x vec))
                        (y (cartesian/y vec))
                        (z (cartesian/z vec)))
                    (let ((rho (sqrt (+ (^2 x) (^2 y) (^2 z)))))
                      (if (zerop rho)
                          +null-vector+
                          (vector rho (atan y x) (asin (/ z rho))))))))
              (lambda (vec) ; to cartesian c.s.
                (let ((rho (spherical/rho vec))
                      (phi (spherical/phi vec))
                      (theta (spherical/theta vec)))
                  (vector (* rho (cos theta) (cos phi))
                          (* rho (cos theta) (sin phi))
                          (* rho (sin theta))))))))

(defun coordinate-system-conversion-function (&key (source-cs :cartesian)
                                                (destination-cs :cartesian))
  (if (eql source-cs destination-cs)
      #'identity
      (let ((src-to-cartesian-conv
             (if (eql source-cs :cartesian)
                 #'identity
                 (let ((fn (getf *coord-system-conversion-fn-plist*
                                 source-cs)))
                   (if fn
                       (cdr fn)
                       (error "COORDINATE-SYSTEM-CONVERSION-FUNCTION -- unsupported or invalid coordinate system given: ~A"
                              source-cs)))))
            (cartesian-to-dest-conv
             (if (eql destination-cs :cartesian)
                 #'identity
                 (let ((fn (getf *coord-system-conversion-fn-plist*
                                 destination-cs)))
                   (if fn
                       (car fn)
                       (error "COORDINATE-SYSTEM-CONVERSION-FUNCTION -- unsupported or invalid coordinate system given: ~A"
                              destination-cs))))))
        (lambda (vec)
          (funcall cartesian-to-dest-conv
                   (funcall src-to-cartesian-conv vec))))))

;;; -------------------------------------------------------------------------

(defun coordinates (&key (cs :cartesian) (v +null-vector+))
  (let ((coord-lambda
         (let (current-cs current-v)
           (lambda (action &key cs v)
             (ecase action
               (get
                (if cs
                    (if (eql t cs)
                        current-cs
                        (funcall (coordinate-system-conversion-function
                                  :source-cs current-cs
                                  :destination-cs cs)
                                 current-v))
                    current-v))
               (set
                (cond
                  ((and (not (null cs))
                      (not (or (eql cs :cartesian)
                            (getf *coord-system-conversion-fn-plist*
                                  cs))))
                   (error "COORDINATES -- unsupported or invalid coordinate system given: ~A"
                          cs))
                  
                  ((and (not (null v)) (not (vectorp v)))
                   (error "COORDINATES -- argument is not a vector: ~A"
                          v))
                  
                  ((and (null cs) (not (null v)))
                   (setf current-v v))
                  
                  ((and (not (null cs)) (null v))
                   (let ((old-cs current-cs))
                     (setf current-cs cs
                        current-v
                        (funcall (coordinate-system-conversion-function
                                  :source-cs old-cs
                                  :destination-cs cs)
                                 current-v))))
                  
                  ((and (not (null cs)) (not (null v)))
                   (setf current-cs cs
                      current-v v)))))))))
    (funcall coord-lambda 'set :cs cs :v v)
    coord-lambda))

(defmacro coordinates/get-coordinate-system (coord)
  `(funcall ,coord 'get :cs t))

(defmacro coordinates/get-vector (coord &optional (cs :cartesian))
  `(funcall ,coord 'get :cs ,cs))

(defmacro coordinates/set-vector (coord vec)
  `(funcall ,coord 'set :v ,vec))

(defmacro coordinates/set-coordinate-system (coord cs)
  `(funcall ,coord 'set :cs ,cs))

(defmacro coordinates/set (coord cs vec)
  `(funcall ,coord 'set :cs ,cs :v ,vec))

;;; -------------------------------------------------------------------------

(defmacro v+ (vec &rest args)
  `(map 'vector #'+ ,vec ,@args))

(defmacro v- (vec &rest args)
  `(map 'vector #'- ,vec ,@args))

(defmacro v* (scalar vec)
  `(map 'vector (lambda (e) (* e ,scalar)) ,vec))

(defmacro v*e (vec1 vec2)
  `(map 'vector #'* ,vec1 ,vec2))

(defmacro v-dot (vec1 vec2)
  `(reduce #'+ (v*e ,vec1 ,vec2)))

(defmacro v-norm (vec)
  (let ((vec-value (gensym)))
    `(let ((,vec-value ,vec))
       (sqrt (v-dot ,vec-value ,vec-value)))))

;;; -------------------------------------------------------------------------

(defun length-of-line-segment (crd1 &optional crd2)
  (let ((vec1 (coordinates/get-vector crd1 :cartesian))
        (vec2 (if crd2 (coordinates/get-vector crd2 :cartesian)
                  +null-vector+)))
    (v-norm (v- vec1 vec2))))

;;; -------------------------------------------------------------------------

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

(defun convert-quaternion-to-matrix3x3 (quat)
  (let ((a (elt quat 0)) (b (elt quat 1)) (c (elt quat 2)) (d (elt quat 3)))
    (let ((a^2 (* a a)) (b^2 (* b b)) (c^2 (* c c)) (d^2 (* d d))
          (ab (* a b)) (ac (* a c)) (ad (* a d))
          (bc (* b c)) (bd (* b d)) (cd (* c d)))
      (let ((norm^2 (+ a^2 b^2 c^2 d^2)))
        (make-array '(3 3) :element-type 'double-float
                    :initial-contents
                    (list (list (/ (+ a^2 b^2 (- c^2) (- d^2)) norm^2)
                                (/ (* 2 (- bc ad)) norm^2)
                                (/ (* 2 (+ ac bd)) norm^2))
                          (list (/ (* 2 (+ bc ad)) norm^2)
                                (/ (+ a^2 (- b^2) c^2 (- d^2)) norm^2)
                                (/ (* 2 (- cd ab)) norm^2))
                          (list (/ (* 2 (- bd ac)) norm^2)
                                (/ (* 2 (+ ab cd)) norm^2)
                                (/ (+ a^2 (- b^2) (- c^2) d^2) norm^2))))))))

;;; -------------------------------------------------------------------------

(defun invert-rotation-matrix (rot-matrix)
  (when rot-matrix
    (let ((inv-rot-matrix (make-array '(3 3) :element-type 'double-float)))
      (setf (aref inv-rot-matrix 0 0) (aref rot-matrix 0 0)
         (aref inv-rot-matrix 0 1) (aref rot-matrix 1 0)
         (aref inv-rot-matrix 0 2) (aref rot-matrix 2 0)
         (aref inv-rot-matrix 1 0) (aref rot-matrix 0 1)
         (aref inv-rot-matrix 1 1) (aref rot-matrix 1 1)
         (aref inv-rot-matrix 1 2) (aref rot-matrix 2 1)
         (aref inv-rot-matrix 2 0) (aref rot-matrix 0 2)
         (aref inv-rot-matrix 2 1) (aref rot-matrix 1 2)
         (aref inv-rot-matrix 2 2) (aref rot-matrix 2 2))
      inv-rot-matrix)))

;;; -------------------------------------------------------------------------

(defun rotate-coordinates-by-matrix (rot-matrix coord)
  (if (null rot-matrix)
      coord
      (let* ((vec (coordinates/get-vector coord))
             (rotated (coordinates
                       :v (multiply-matrix3x3-by-vector3 rot-matrix vec))))
        (coordinates/set-coordinate-system
         rotated (coordinates/get-coordinate-system coord))
        rotated)))

(defun rotate-coordinates-by-quaternion (quat coord)
  (rotate-coordinates-by-matrix (convert-quaternion-to-matrix3x3 quat)
                                coord))
