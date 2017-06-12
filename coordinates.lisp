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

(defmacro v-dot (vec1 vec2)
  `(reduce #'+ (map 'vector #'* ,vec1 ,vec2)))

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
