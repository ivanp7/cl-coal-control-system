;;;; movements.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defparameter *sphere/position-error* 5.0d-2)
(defparameter *sphere/angle-error* (* 5.0d0 (/ pi 180)))

;;; -------------------------------------------------------------------------
;;; No motion

(defun sphere/get-wait-action (delay)
  (let ((steps-to-wait (ceiling (/ delay *simm/simulation-step*)))
        (counter 0))
    (lambda (&rest args)
      (declare (ignore args))
      (if (< counter steps-to-wait)
          (progn
            (incf counter)
            nil)
          (progn
            (setf counter 0)
            t)))))

;;; -------------------------------------------------------------------------
;;; Smooth linear motion

(defun sphere/get-linear-motion-action
    (velocity new-pos &optional (target-ref-frame/world-p
                                 (point/reference-frame-world-p new-pos)))
  (let ((step-distance (* velocity *simm/simulation-step*))
        (new-pos-v (coordinates/get-vector
                    (point/coordinates
                     (sphere/transform-reference-frame
                      new-pos
                      target-ref-frame/world-p)))))
    (lambda (&rest args)
      (declare (ignore args))
      (let* ((current-pos-v
              (coordinates/get-vector
               (point/coordinates (sphere/get-endeffector-position
                                   target-ref-frame/world-p))))
             (line-v (v- new-pos-v current-pos-v))
             (remaining-distance (v-norm line-v))
             (direction-v (v* (/ remaining-distance) line-v)))
        (sphere/set-endeffector-position
         (if (<= remaining-distance step-distance)
             new-pos
             (let ((next-pos-v (v+ current-pos-v
                                   (v* step-distance direction-v))))
               (point (coordinates :v next-pos-v)
                      target-ref-frame/world-p)))
         target-ref-frame/world-p)
        (< remaining-distance *sphere/position-error*)))))

;;; -------------------------------------------------------------------------
;;; Rolling cycle motion

(defparameter *sphere/cycle-motion/cos-rule/amplitude* 1.0d0)
(defparameter *sphere/cycle-motion/cos-rule/bias* 0.0d0)
(defparameter *sphere/cycle-motion/cos-rule/exponent* 0.5d0)
(defun sphere/cycle-motion/get-cos-rule
    (&key (amplitude *sphere/cycle-motion/cos-rule/amplitude*)
       (bias *sphere/cycle-motion/cos-rule/bias*)
       (exponent *sphere/cycle-motion/cos-rule/exponent*))
  (lambda (theta) (* amplitude (+ bias (* (- 1.0d0 bias) (expt (cos theta) exponent))))))

(defparameter *sphere/cycle-motion/theta-offset* (* pi 5/180))

(defun sphere/cycle-motion/get-pole-theta (pole-point azimuth)
  (let* ((pole-v (coordinates/get-vector
                  (point/coordinates
                   (sphere/transform-reference-frame pole-point t))
                  :spherical))
         (pole-az (spherical/phi pole-v))
         (pole-elev (spherical/theta pole-v)))
    (if (or (<= (abs (- azimuth pole-az)) (/ pi 2))
           (>= (abs (- azimuth pole-az)) (* pi 3/2)))
        pole-elev
        (- pole-elev))))

(defun sphere/get-rolling-cycle-motion-action
    (azimuth stop-fn &optional (displacement-rule
                                (sphere/cycle-motion/get-cos-rule)))
  (let (pole-point)
    (simm/get-custom-motion-action
     :initial-fn
     (lambda (&rest args)
       (declare (ignore args))
       (let* ((ee-pos-v (coordinates/get-vector
                         (point/coordinates
                          (sphere/get-endeffector-position t))
                         :spherical))
              (pole-v
               (vector 1.0d0
                       azimuth
                       (let ((theta-threshold (- (/ pi 2)
                                             *sphere/cycle-motion/theta-offset*)))
                         (cond ((< (spherical/rho ee-pos-v)
                                   *sphere/position-error*)
                                theta-threshold)
                               ((> (spherical/theta ee-pos-v)
                                   theta-threshold)
                                theta-threshold)
                               ((< (spherical/theta ee-pos-v)
                                   (- theta-threshold))
                                (- theta-threshold))
                               (t (spherical/theta ee-pos-v)))))))
         (setf pole-point
            (sphere/transform-reference-frame
             (point (coordinates :cs :spherical :v pole-v) t) nil))))
     :stop-test-fn
     (lambda (&rest args)
       (declare (ignore args))
       (and pole-point
          (let ((theta (sphere/cycle-motion/get-pole-theta
                    pole-point azimuth)))
            (funcall stop-fn (funcall displacement-rule theta) theta))))
     :iteration-fn
     (lambda (&rest args)
       (declare (ignore args))
       (sphere/set-endeffector-position
        (point (coordinates :cs :spherical
                            :v (let ((theta (sphere/cycle-motion/get-pole-theta
                                         pole-point azimuth)))
                                 (vector (funcall displacement-rule theta)
                                         azimuth
                                         theta)))
               t)))
     :final-fn
     (lambda (&rest args)
       (declare (ignore args))
       (setf pole-point nil)))))

;;; -------------------------------------------------------------------------
;;; Quick stop motion

(defun sphere/quick-stop/critical-velocity ()
  (let* ((coef (+ (* 1/2 (1+ *sphere/endeffector-inertia-coefficient*)
                     (/ *sphere/mass* *sphere/endeffector-mass*))
                  (* 1/3 (expt (/ *sphere/endeffector-radius*
                               *sphere/radius*) 2))
                  (* 1/2 (expt (- 1 *sphere/max-endeffector-relative-displacement*)
                            2)))))
    (sqrt (/ (* (v-norm *simm/gravity-vector*)
             *sphere/max-endeffector-relative-displacement*
             *sphere/radius*)
          coef))))

(defparameter *sphere/quick-stop/theta-offset* (* pi 5/180))
(defparameter *sphere/quick-stop/velocity-threshold*
  (/ (sphere/quick-stop/critical-velocity) 2))

(defun sphere/get-quick-stop-motion-action
    (&optional (velocity-threshold *sphere/quick-stop/velocity-threshold*))
  (simm/get-custom-motion-action
   :stop-test-fn
   (lambda (&rest args)
     (let* ((body/state (nth 2 args))
            (velocity (pb/simm/body-state/velocity body/state))
            (velocity-magn
             (sqrt (+ (expt (pb/simm/vector3/element velocity 2) 2)
                   (expt (pb/simm/vector3/element velocity 1) 2)))))
       (< velocity-magn velocity-threshold)))
   :initial-fn
   (lambda (&rest args)
     (let* ((body/state (nth 2 args))
            (velocity (pb/simm/body-state/velocity body/state))
            (velocity-azimuth
             (atan (pb/simm/vector3/element velocity 2)
                   (pb/simm/vector3/element velocity 1))))
       (sphere/set-endeffector-position
        (point (coordinates :cs :spherical
                            :v (vector 1.0d0
                                       velocity-azimuth
                                       (/ pi -2))) t) nil)))
   :iteration-fn
   (lambda (&rest args)
     (let* ((current-pos-v (coordinates/get-vector
                            (point/coordinates
                             (sphere/get-endeffector-position t))
                            :spherical))
            (body/state (nth 2 args))
            (velocity (pb/simm/body-state/velocity body/state))
            (velocity-azimuth
             (atan (pb/simm/vector3/element velocity 2)
                   (pb/simm/vector3/element velocity 1))))
       (when (plusp (spherical/theta current-pos-v))
         (sphere/set-endeffector-position
          (point (coordinates :cs :spherical
                              :v (vector 1.0d0
                                         (+ velocity-azimuth pi)
                                         *sphere/quick-stop/theta-offset*))
                 t) nil))))))
