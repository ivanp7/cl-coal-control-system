;;;; pid-controller.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defun pid-controller (&key (kp 0.0d0) (ki 0.0d0) (kd 0.0d0)
                         (initial-process-time 0.0d0)
                         (initial-process-value 0.0d0)
                         (initial-error-value 0.0d0)
                         (initial-error-integral 0.0d0)
                         (initial-error-derivative 0.0d0)
                         (output-saturation-fn #'identity))
  (let ((previous-process-time initial-process-time)
        (previous-process-value initial-process-value)
        (previous-error-value initial-error-value)
        (previous-error-integral initial-error-integral)
        (previous-error-derivative initial-error-derivative))
    (lambda (process-time process-value setpoint)
      (let* ((delta-t (- process-time previous-process-time))
             (error-value (- setpoint process-value))
             (error-integral (+ previous-error-integral
                                (* 1/2 delta-t (+ error-value previous-error-value))))
             (error-derivative (if (not (zerop delta-t))
                                   (/ (- error-value previous-error-value)
                                      (- process-time previous-process-time))
                                   0.0d0)))
        (setf previous-process-time process-time
           previous-process-value process-value
           previous-error-value error-value
           previous-error-integral error-integral
           previous-error-derivative error-derivative)
        (funcall output-saturation-fn
                 (+ (* kp error-value)
                    (* ki error-integral)
                    (* kd error-derivative)))))))
