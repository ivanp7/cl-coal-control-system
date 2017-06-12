;;;; spherical-robot.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

;;; -------------------------------------------------------------------------
;;; Spherial robot configuration parameters

(defparameter *sphere/number-of-legs* 3)

(defparameter *sphere/legs-setpoints-calculation-function*
  (lambda (endeffector-position) ; default: portal mechanism
    endeffector-position))
(defparameter *sphere/max-endeffector-relative-displacement* 0.5d0)

(defparameter *sphere/radius* 1.0d0) ; meters
(defparameter *sphere/endeffector-radius* 0.2d0) ; meters

(defparameter *sphere/mass* 1.0d0)
(defparameter *sphere/endeffector-mass* 10.0d0)

(defparameter *sphere/endeffector-inertia-coefficient* 2/5)

(defparameter *sphere/battery-recharge-allowed* nil)

;;; -------------------------------------------------------------------------
;;; Prismatic joint PID controller parameters

(defparameter *sphere/leg-pid-controller-kp* 3.0d2)
(defparameter *sphere/leg-pid-controller-ki* 1.0d0)
(defparameter *sphere/leg-pid-controller-kd* 3.0d1)
(defparameter *sphere/leg-pid-controller-force-limit* 1.0d3)

;;; -------------------------------------------------------------------------
;;; End-effector functionality

(defparameter *sphere/endeff-pos* (point (coordinates) nil))

(defparameter *sphere/body-cs-rotation-matrix* ())
(defparameter *sphere/body-cs-inverse-rotation-matrix* ())

(defun sphere/transform-reference-frame (pnt target-rf/world-p)
  (transform-reference-frame pnt
                             target-rf/world-p
                             *sphere/body-cs-rotation-matrix*
                             *sphere/body-cs-inverse-rotation-matrix*))

(defun sphere/get-endeffector-position
    (&optional (target-ref-frame/world-p (point/reference-frame-world-p
                                          *sphere/endeff-pos*)))
  (sphere/transform-reference-frame *sphere/endeff-pos*
                                    target-ref-frame/world-p))

(defun sphere/set-endeffector-position
    (new-pos &optional (target-ref-frame/world-p
                        (point/reference-frame-world-p new-pos)))
  (setf *sphere/endeff-pos*
     (sphere/transform-reference-frame new-pos
                                       target-ref-frame/world-p)))

;;; -------------------------------------------------------------------------
;;; Body-environment interaction functionality [TODO]

(defparameter *sphere/body-environment-interaction-function*
  (lambda (model-time body/reaction body/state)
    (declare (ignore model-time body/reaction))
    (let ((angular-velocity (pb/simm/body-state/angular-velocity body/state)))
      angular-velocity)))

;;; -------------------------------------------------------------------------
;;; End-effector motion scenario

(defun sphere/get-simple-endeffector-position-changing-action
    (new-pos &optional (target-ref-frame/world-p
                        (point/reference-frame-world-p new-pos)))
  (lambda (&rest args)
    (declare (ignore args))
    (sphere/set-endeffector-position new-pos target-ref-frame/world-p)
    t))

(defparameter *sphere/endeffector-motion-scenario*
  (finite-state-machine
   (list (new-control-scenario-state-descriptor :idle))))

;;; -------------------------------------------------------------------------
;;; Spherical robot control system

(defparameter *sphere/model-time* 0.0d0)
(defparameter *sphere/actuators-power* 0.0d0)
(defparameter *sphere/energy-spent* 0.0d0)

(defun sphere/count-energy-consumption (model-time legs/state)
  (let ((prev-model-time *sphere/model-time*)
        (prev-power *sphere/actuators-power*)
        (power 0.0d0))
    (dotimes (i *sphere/number-of-legs*)
      (setf power (+ power
                  (let ((delta-power
                         (* (pb/simm/prismatic-joint-state/computed-force
                             (elt legs/state i))
                            (pb/simm/prismatic-joint-state/velocity
                             (elt legs/state i)))))
                    (if (plusp delta-power)
                        delta-power
                        (if *sphere/battery-recharge-allowed*
                            delta-power
                            0.0d0))))))
    (setf *sphere/energy-spent*
       (+ *sphere/energy-spent*
          (* (- model-time prev-model-time)
             (* 1/2 (+ power prev-power)))))
    (setf *sphere/model-time* model-time
       *sphere/actuators-power* power)))

;;; -------------------------------------------------------------------------
;;; End-effector position control functionality

(defparameter *sphere/leg-pid-controllers* nil)

(defun sphere/endeffector-control (model-time legs/reaction legs/state)
  (declare (ignorable model-time legs/state))
  (let ((legs-setpoints
         (funcall *sphere/legs-setpoints-calculation-function*
                  (v* *sphere/max-endeffector-relative-displacement*
                      (coordinates/get-vector
                       (point/coordinates
                        (sphere/get-endeffector-position nil)))))))
    (dotimes (i *sphere/number-of-legs*)
      (setf (pb/simm/prismatic-joint-reaction/output (elt legs/reaction i))
         (funcall (elt *sphere/leg-pid-controllers* i)
                  model-time
                  (pb/simm/prismatic-joint-state/position
                   (elt legs/state i))
                  (elt legs-setpoints i))))))

;;; -------------------------------------------------------------------------
;;; Spherical robot control system

(defun sphere/control-system
    (model-time body/reaction legs/reaction endeffector/reaction
     body/state legs/state endeffector/state)
  (declare (ignore endeffector/reaction))
  
  (let ((legs/reaction
         (map 'vector
              (lambda (part)
                (pb/simm/mechanism-part/reaction
                 (pb/simm/mechanism-part/prismatic-joint part)))
              legs/reaction))
        (legs/state
         (map 'vector
              (lambda (part)
                (pb/simm/mechanism-part/state
                 (pb/simm/mechanism-part/prismatic-joint part)))
              legs/state)))
    (setf *sphere/body-cs-rotation-matrix*
       (simm/matrix3x3-object-to-matrix
        (pb/simm/body-state/rotation-matrix body/state)))
    (setf *sphere/body-cs-inverse-rotation-matrix*
       (invert-rotation-matrix *sphere/body-cs-rotation-matrix*))
    
    (funcall *sphere/body-environment-interaction-function*
             model-time body/reaction body/state)
    (funcall *sphere/endeffector-motion-scenario* 'proceed
             model-time legs/reaction body/state legs/state endeffector/state)
    (sphere/endeffector-control model-time legs/reaction legs/state)
    
    (sphere/count-energy-consumption model-time legs/state)
    (list *sphere/energy-spent* *sphere/actuators-power*)))

(defun sphere/reset-control-system (cs-reaction-msg)
  (setf *sphere/leg-pid-controllers*
     (let ((pids (make-array *sphere/number-of-legs*
                             :element-type 'function)))
       (dotimes (i *sphere/number-of-legs*)
         (setf (elt pids i)
            (pid-controller
             :kp *sphere/leg-pid-controller-kp*
             :ki *sphere/leg-pid-controller-ki*
             :kd *sphere/leg-pid-controller-kd*
             :output-saturation-fn
             (lambda (force)
               (cond
                 ((> force *sphere/leg-pid-controller-force-limit*)
                  *sphere/leg-pid-controller-force-limit*)
                 ((< force (- *sphere/leg-pid-controller-force-limit*))
                  (- *sphere/leg-pid-controller-force-limit*))
                 (t force))))))
       pids))
  
  (setf *sphere/endeff-pos* (point (coordinates) nil))
  
  (setf *sphere/model-time* 0.0d0
     *sphere/actuators-power* 0.0d0
     *sphere/energy-spent* 0.0d0
     (pb/obj-component/raw-vector
      (elt (pb/msg/components cs-reaction-msg) 2))
     (pb/obj-component/new-double-value-raw-vector #(0.0d0 0.0d0))))

(defparameter *control-system-function/simm/spherical-robot*
  (lambda (obj-state-msg cs-reaction-msg)
    (if (eql (pb/msg/direction obj-state-msg)
             +pb/direction/initial-from-obj-to-cs+)
        (sphere/reset-control-system cs-reaction-msg)
        (let ((model-time
               (pb/raw-vector-element/double-value
                (elt (pb/raw-vector/elements
                      (pb/obj-component/raw-vector
                       (elt (pb/msg/components obj-state-msg) 0))) 0)))
              (mech-parts/state
               (pb/simm/mechanism/parts
                (pb/obj-component/simm-mechanism
                 (elt (pb/msg/components obj-state-msg) 1))))
              (mech-parts/reaction
               (pb/simm/mechanism/parts
                (pb/obj-component/simm-mechanism
                 (elt (pb/msg/components cs-reaction-msg) 1))))
              (output-vector
               (pb/raw-vector/elements
                (pb/obj-component/raw-vector
                 (elt (pb/msg/components cs-reaction-msg) 2)))))
          
          (if (< (length mech-parts/state) (+ 2 *sphere/number-of-legs*))
              (error "*CONTROL-SYSTEM-FUNCTION* [sphere] -- number of mechanism parts is too small or value of *SPHERE/NUMBER-OF-LEGS* is incorrect. ")
              (let ((body/state
                     (pb/simm/mechanism-part/state
                      (pb/simm/mechanism-part/body
                       (elt mech-parts/state 0))))
                    (body/reaction
                     (pb/simm/mechanism-part/reaction
                      (pb/simm/mechanism-part/body
                       (elt mech-parts/reaction 0))))
                    
                    (endeffector/state
                     (pb/simm/mechanism-part/state
                      (pb/simm/mechanism-part/body
                       (elt mech-parts/state
                            (1- (length mech-parts/state))))))
                    (endeffector/reaction
                     (pb/simm/mechanism-part/reaction
                      (pb/simm/mechanism-part/body
                       (elt mech-parts/reaction
                            (1- (length mech-parts/reaction))))))
                    
                    (legs/state
                     (subseq mech-parts/state
                             1 (1- (length mech-parts/state))))
                    (legs/reaction
                     (subseq mech-parts/reaction
                             1 (1- (length mech-parts/reaction)))))
                
                (loop for i = 0 then (1+ i)
                   for value in
                     (sphere/control-system
                      model-time
                      body/reaction legs/reaction endeffector/reaction
                      body/state legs/state endeffector/state)
                   do
                     (setf (pb/raw-vector-element/double-value
                         (elt output-vector i))
                        value))))))
    cs-reaction-msg))
