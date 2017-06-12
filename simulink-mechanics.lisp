;;;; simulink-mechanics.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defmacro pb/simm/vector3/element (obj index)
  (if (and (numberp index) (>= index 1) (<= index 3))
      (case index
        (1 `(com-ivanp7-cscp.simulink-mechanics:e1 ,obj))
        (2 `(com-ivanp7-cscp.simulink-mechanics:e2 ,obj))
        (3 `(com-ivanp7-cscp.simulink-mechanics:e3 ,obj)))
      `(error ,(format nil "PB/SIMM/VECTOR3/ELEMENT -- invalid index: ~A"
                       index))))

(defmacro pb/simm/matrix3x3/row (obj index)
  (if (and (numberp index) (>= index 1) (<= index 3))
      (case index
        (1 `(com-ivanp7-cscp.simulink-mechanics:row1 ,obj))
        (2 `(com-ivanp7-cscp.simulink-mechanics:row2 ,obj))
        (3 `(com-ivanp7-cscp.simulink-mechanics:row3 ,obj)))
      `(error ,(format nil "PB/SIMM/MATRIX3X3/ROW -- invalid index: ~A"
                       index))))

(defmacro pb/simm/matrix3x3/element (obj index-row index-column)
  `(pb/simm/vector3/element
    (pb/simm/matrix3x3/row ,obj ,index-row) ,index-column))

(defun simm/vector3-object-to-vector (obj)
  (let ((vec (make-array 3 :element-type 'double-float)))
    (setf (elt vec 0) (pb/simm/vector3/element obj 1)
       (elt vec 1) (pb/simm/vector3/element obj 2)
       (elt vec 2) (pb/simm/vector3/element obj 3))
    vec))

(defun simm/matrix3x3-object-to-matrix (obj)
  (let ((mat (make-array '(3 3) :element-type 'double-float)))
    (setf (aref mat 0 0) (pb/simm/matrix3x3/element obj 1 1)
       (aref mat 0 1) (pb/simm/matrix3x3/element obj 1 2)
       (aref mat 0 2) (pb/simm/matrix3x3/element obj 1 3)
       (aref mat 1 0) (pb/simm/matrix3x3/element obj 2 1)
       (aref mat 1 1) (pb/simm/matrix3x3/element obj 2 2)
       (aref mat 1 2) (pb/simm/matrix3x3/element obj 2 3)
       (aref mat 2 0) (pb/simm/matrix3x3/element obj 3 1)
       (aref mat 2 1) (pb/simm/matrix3x3/element obj 3 2)
       (aref mat 2 2) (pb/simm/matrix3x3/element obj 3 3))
    mat))

;;; -------------------------------------------------------------------------

(defun pb/simm/new-mechanism-parts-array (size)
  (let ((arr (make-array size :adjustable t :fill-pointer 0)))
    (dotimes (i size)
      (vector-push
       (make-instance 'com-ivanp7-cscp.simulink-mechanics:mechanism-part)
       arr))
    arr))

(defun pb/simm/new-vector3 (&optional (vec +null-vector+))
  (let ((vector3 (make-instance 'com-ivanp7-cscp.simulink-mechanics:vector3)))
    (setf (pb/simm/vector3/element vector3 1) (elt vec 0)
       (pb/simm/vector3/element vector3 2) (elt vec 1)
       (pb/simm/vector3/element vector3 3) (elt vec 2))
    vector3))

;;; -------------------------------------------------------------------------

(defmacro pb/obj-component/has-simm-mechanism (obj)
  `(com-ivanp7-cscp:has-simulink-mechanics-mechanism ,obj))

(defmacro pb/obj-component/simm-mechanism (obj)
  `(com-ivanp7-cscp:simulink-mechanics-mechanism ,obj))

(defmacro pb/simm/mechanism/parts (obj)
  `(com-ivanp7-cscp.simulink-mechanics:parts ,obj))

;;; -------------------------------------------------------------------------

(defmacro pb/simm/mechanism-part/name (obj)
  `(com-ivanp7-cscp.simulink-mechanics:name ,obj))

(defmacro pb/simm/mechanism-part/has-body (obj)
  `(com-ivanp7-cscp.simulink-mechanics:has-body ,obj))

(defmacro pb/simm/mechanism-part/body (obj)
  `(com-ivanp7-cscp.simulink-mechanics:body ,obj))

(defmacro pb/simm/mechanism-part/has-weld-joint (obj)
  `(com-ivanp7-cscp.simulink-mechanics:has-weld-joint ,obj))

(defmacro pb/simm/mechanism-part/weld-joint (obj)
  `(com-ivanp7-cscp.simulink-mechanics:weld-joint ,obj))

(defmacro pb/simm/mechanism-part/has-prismatic-joint (obj)
  `(com-ivanp7-cscp.simulink-mechanics:has-prismatic-joint ,obj))

(defmacro pb/simm/mechanism-part/prismatic-joint (obj)
  `(com-ivanp7-cscp.simulink-mechanics:prismatic-joint ,obj))

(defmacro pb/simm/mechanism-part/has-revolute-joint (obj)
  `(com-ivanp7-cscp.simulink-mechanics:has-revolute-joint ,obj))

(defmacro pb/simm/mechanism-part/revolute-joint (obj)
  `(com-ivanp7-cscp.simulink-mechanics:revolute-joint ,obj))

;;; -------------------------------------------------------------------------

(defmacro pb/simm/mechanism-part/has-state (obj)
  `(com-ivanp7-cscp.simulink-mechanics:has-state ,obj))

(defmacro pb/simm/mechanism-part/state (obj)
  `(com-ivanp7-cscp.simulink-mechanics:state ,obj))

(defmacro pb/simm/mechanism-part/has-reaction (obj)
  `(com-ivanp7-cscp.simulink-mechanics:has-reaction ,obj))

(defmacro pb/simm/mechanism-part/reaction (obj)
  `(com-ivanp7-cscp.simulink-mechanics:reaction ,obj))

;;; -------------------------------------------------------------------------

(defmacro pb/simm/body-state/position (obj)
  `(com-ivanp7-cscp.simulink-mechanics:position ,obj))

(defmacro pb/simm/body-state/velocity (obj)
  `(com-ivanp7-cscp.simulink-mechanics:velocity ,obj))

(defmacro pb/simm/body-state/angular-velocity (obj)
  `(com-ivanp7-cscp.simulink-mechanics:angular-velocity ,obj))

(defmacro pb/simm/body-state/rotation-matrix (obj)
  `(com-ivanp7-cscp.simulink-mechanics:rotation-matrix ,obj))

(defmacro pb/simm/body-state/acceleration (obj)
  `(com-ivanp7-cscp.simulink-mechanics:acceleration ,obj))

(defmacro pb/simm/body-state/angular-acceleration (obj)
  `(com-ivanp7-cscp.simulink-mechanics:angular-acceleration ,obj))

(defmacro pb/simm/body-reaction/torque (obj)
  `(com-ivanp7-cscp.simulink-mechanics:torque ,obj))

(defmacro pb/simm/body-reaction/force (obj)
  `(com-ivanp7-cscp.simulink-mechanics:force ,obj))

;;; -------------------------------------------------------------------------

(defmacro pb/simm/weld-joint-state/reaction-force (obj)
  `(com-ivanp7-cscp.simulink-mechanics:reaction-force ,obj))

(defmacro pb/simm/weld-joint-state/reaction-torque (obj)
  `(com-ivanp7-cscp.simulink-mechanics:reaction-torque ,obj))

;;; -------------------------------------------------------------------------

(defmacro pb/simm/prismatic-joint-state/position (obj)
  `(com-ivanp7-cscp.simulink-mechanics:position ,obj))

(defmacro pb/simm/prismatic-joint-state/velocity (obj)
  `(com-ivanp7-cscp.simulink-mechanics:velocity ,obj))

(defmacro pb/simm/prismatic-joint-state/acceleration (obj)
  `(com-ivanp7-cscp.simulink-mechanics:acceleration ,obj))

(defmacro pb/simm/prismatic-joint-state/computed-force (obj)
  `(com-ivanp7-cscp.simulink-mechanics:computed-force ,obj))

(defmacro pb/simm/prismatic-joint-state/reaction-torque (obj)
  `(com-ivanp7-cscp.simulink-mechanics:reaction-torque ,obj))

(defmacro pb/simm/prismatic-joint-state/reaction-force (obj)
  `(com-ivanp7-cscp.simulink-mechanics:reaction-force ,obj))

(defmacro pb/simm/prismatic-joint-reaction/output (obj)
  `(com-ivanp7-cscp.simulink-mechanics:output ,obj))

;;; -------------------------------------------------------------------------

(defmacro pb/simm/revolute-joint-state/angle (obj)
  `(com-ivanp7-cscp.simulink-mechanics:angle ,obj))

(defmacro pb/simm/revolute-joint-state/angular-velocity (obj)
  `(com-ivanp7-cscp.simulink-mechanics:angular-velocity ,obj))

(defmacro pb/simm/revolute-joint-state/angular-acceleration (obj)
  `(com-ivanp7-cscp.simulink-mechanics:angular-acceleration ,obj))

(defmacro pb/simm/revolute-joint-state/computed-torque (obj)
  `(com-ivanp7-cscp.simulink-mechanics:computed-torque ,obj))

(defmacro pb/simm/revolute-joint-state/reaction-torque (obj)
  `(com-ivanp7-cscp.simulink-mechanics:reaction-torque ,obj))

(defmacro pb/simm/revolute-joint-state/reaction-force (obj)
  `(com-ivanp7-cscp.simulink-mechanics:reaction-force ,obj))

(defmacro pb/simm/revolute-joint-reaction/output (obj)
  `(com-ivanp7-cscp.simulink-mechanics:output ,obj))

;;; -------------------------------------------------------------------------

(defun simm/reset-body-reaction (body)
  (setf (pb/simm/body-reaction/torque (pb/simm/mechanism-part/reaction body))
     (pb/simm/new-vector3))
  (setf (pb/simm/body-reaction/force (pb/simm/mechanism-part/reaction body))
     (pb/simm/new-vector3))
  t)

(defun simm/reset-weld-joint-reaction (weld-joint)
  (pb/simm/mechanism-part/reaction weld-joint)
  t)

(defun simm/reset-prismatic-joint-reaction (prismatic-joint)
  (setf (pb/simm/prismatic-joint-reaction/output
      (pb/simm/mechanism-part/reaction prismatic-joint))
     0.0d0)
  t)

(defun simm/reset-revolute-joint-reaction (revolute-joint)
  (setf (pb/simm/revolute-joint-reaction/output
      (pb/simm/mechanism-part/reaction revolute-joint))
     0.0d0)
  t)

(defun simm/reset-mechanism-reaction (mech/state mech/reaction)
  (when (or (null (pb/simm/mechanism/parts mech/reaction))
           (/= (length (pb/simm/mechanism/parts mech/reaction))
              (length (pb/simm/mechanism/parts mech/state))))
    (setf (pb/simm/mechanism/parts mech/reaction)
       (pb/simm/new-mechanism-parts-array
        (length (pb/simm/mechanism/parts mech/state)))))
  
  (loop
     for mech-part/state across (pb/simm/mechanism/parts mech/state)
     for mech-part/reaction across (pb/simm/mechanism/parts mech/reaction)
     do
       (progn
         (setf (pb/simm/mechanism-part/name mech-part/reaction)
            (pb/simm/mechanism-part/name mech-part/state))
         
         (cond
           ((pb/simm/mechanism-part/has-body mech-part/state)
            (simm/reset-body-reaction
             (pb/simm/mechanism-part/body
              mech-part/reaction)))
           
           ((pb/simm/mechanism-part/has-weld-joint mech-part/state)
            (simm/reset-weld-joint-reaction
             (pb/simm/mechanism-part/weld-joint
              mech-part/reaction)))
           
           ((pb/simm/mechanism-part/has-prismatic-joint mech-part/state)
            (simm/reset-prismatic-joint-reaction
             (pb/simm/mechanism-part/prismatic-joint
              mech-part/reaction)))
           
           ((pb/simm/mechanism-part/has-revolute-joint mech-part/state)
            (simm/reset-revolute-joint-reaction
             (pb/simm/mechanism-part/revolute-joint
              mech-part/reaction))))))
  t)

;;; -------------------------------------------------------------------------

(let ((entry (assoc :simm *pb/obj-component-reaction-resetters-alist*))
      (resetter
       (lambda (obj-component/state obj-component/reaction)
         (when (pb/obj-component/has-simm-mechanism obj-component/state)
           (simm/reset-mechanism-reaction
            (pb/obj-component/simm-mechanism obj-component/state)
            (pb/obj-component/simm-mechanism obj-component/reaction))))))
  (if entry
      (rplacd entry resetter)
      (setf *pb/obj-component-reaction-resetters-alist*
         (acons :simm resetter *pb/obj-component-reaction-resetters-alist*))))

(defparameter *control-system-function/simm* ;; dummy control system
  (lambda (obj-state-msg cs-reaction-msg)
    (format t "~A~%" (pb/raw-vector-element/double-value
                      (elt (pb/raw-vector/elements
                            (pb/obj-component/raw-vector
                             (elt (pb/msg/components obj-state-msg) 0))) 0)))
    cs-reaction-msg))

;;; -------------------------------------------------------------------------

(defun simm/get-custom-motion-action
    (&key initial-fn (stop-test-fn (constantly t))
       (iteration-fn (constantly nil)) final-fn)
  (let ((initial-op-needed t))
    (lambda (&rest args)
      (when initial-op-needed
        (when initial-fn (apply initial-fn args))
        (setf initial-op-needed nil))
      (if (not (apply stop-test-fn args))
          (progn
            (apply iteration-fn args)
            nil)
          (progn
            (when final-fn (apply final-fn args))
            (setf initial-op-needed t)
            t)))))

;;; -------------------------------------------------------------------------

(defparameter *simm/simulation-step* 0.001d0)

(defparameter *simm/gravity-vector* #(0.0d0 0.0d0 -9.81d0))
