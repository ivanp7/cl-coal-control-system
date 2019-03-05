;;;; fsm.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defun new-fsm-state-descriptor (state fn)
  (cons state fn))

(defmacro fsm-state-descriptor/label (state-descr)
  `(car ,state-descr))

(defmacro fsm-state-descriptor/function (state-descr)
  `(cdr ,state-descr))

(defun finite-state-machine
    (state-descriptor-list &key (initial-state (fsm-state-descriptor/label
                                                (first state-descriptor-list)))
                             (label-test-fn #'eql))
  (let ((state-descriptor-list (copy-tree state-descriptor-list))
        (current-state initial-state))
    (lambda (action &rest args)
      (ecase action
        (get-state-descriptor-list
         (if (null args)
             state-descriptor-list
             (error "FINITE-STATE-MACHINE -- incorrect number of arguments provided.")))
        (set-state-descriptor-list
         (if (= 1 (length args))
             (setf state-descriptor-list (first args))
             (error "FINITE-STATE-MACHINE -- incorrect number of arguments provided.")))
        (get-state
         (if (null args)
             current-state
             (error "FINITE-STATE-MACHINE -- incorrect number of arguments provided.")))
        (set-state
         (if (= 1 (length args))
             (setf current-state (first args))
             (error "FINITE-STATE-MACHINE -- incorrect number of arguments provided.")))
        (proceed
         (let ((state-descr (assoc current-state state-descriptor-list
                                   :test label-test-fn)))
           (if (not (null state-descr))
               (setf current-state (apply (fsm-state-descriptor/function state-descr)
                                       args))
               (error "FINITE-STATE-MACHINE -- cannot proceed: current state ~A is invalid."
                      current-state))))))))
