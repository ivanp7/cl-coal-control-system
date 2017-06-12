;;;; control-scenario.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defun new-control-scenario-state-descriptor
    (state &optional (state-action-fn (constantly t))
             (state-transition-fn (constantly state)))
  (new-fsm-state-descriptor
   state
   (lambda (&rest args)
     (if (not (apply state-action-fn args))
         state
         (apply state-transition-fn args)))))
