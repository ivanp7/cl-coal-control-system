;;;; protocol-buffers.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defparameter *control-system-channel-id* 0)

;;; -------------------------------------------------------------------------

(defparameter *pb/obj-component-reaction-resetters-alist* ())

(defparameter *control-system-function* ;; dummy control system function
  (lambda (obj-state-msg cs-reaction-msg)
    (declare (ignore obj-state-msg))
    cs-reaction-msg))

;;; -------------------------------------------------------------------------

(defconstant +pb/direction/initial-from-obj-to-cs+
  com-ivanp7-cscp:+direction-initial-from-object-to-control-system+)
(defconstant +pb/direction/from-obj-to-cs+
  com-ivanp7-cscp:+direction-from-object-to-control-system+)
(defconstant +pb/direction/from-cs-to-obj+
  com-ivanp7-cscp:+direction-from-control-system-to-object+)

(defun pb/new-message ()
  (make-instance 'com-ivanp7-cscp:control-system-message))

(defun pb/new-object-component-array (size)
  (let ((arr (make-array size :adjustable t :fill-pointer 0)))
    (dotimes (i size)
      (vector-push
       (make-instance 'com-ivanp7-cscp:control-object-component)
       arr))
    arr))

(defmacro pb/msg/description (arg)
  `(com-ivanp7-cscp:description ,arg))

(defmacro pb/msg/channel-id (arg)
  `(com-ivanp7-cscp:communication-channel-id ,arg))

(defmacro pb/msg/direction (arg)
  `(com-ivanp7-cscp:direction ,arg))

(defmacro pb/msg/age (arg)
  `(com-ivanp7-cscp:age ,arg))

(defmacro pb/msg/components (arg)
  `(com-ivanp7-cscp:components ,arg))

(defmacro pb/obj-component/description (arg)
  `(com-ivanp7-cscp:description ,arg))

(defmacro pb/obj-component/prefix-name (arg)
  `(com-ivanp7-cscp:prefix-name ,arg))

(defmacro pb/obj-component/name (arg)
  `(com-ivanp7-cscp:name ,arg))

;;; -------------------------------------------------------------------------

(defmacro pb/obj-component/has-raw-vector (arg)
  `(com-ivanp7-cscp:has-raw-vector ,arg))

(defmacro pb/obj-component/raw-vector (arg)
  `(com-ivanp7-cscp:raw-vector ,arg))

(defmacro pb/raw-vector/elements (arg)
  `(com-ivanp7-cscp:elements ,arg))

(defmacro pb/raw-vector-element/has-double-value (arg)
  `(com-ivanp7-cscp:has-double-value ,arg))

(defmacro pb/raw-vector-element/double-value (arg)
  `(com-ivanp7-cscp:double-value ,arg))

(defun pb/obj-component/new-double-value-raw-vector (vec)
  (let ((raw (make-instance 'com-ivanp7-cscp:raw-vector))
        (elts-arr (make-array (length vec) :adjustable t :fill-pointer 0)))
    (dotimes (i (length vec))
      (vector-push (let ((el (make-instance
                              'com-ivanp7-cscp:raw-vector-element)))
                     (setf (pb/raw-vector-element/double-value el) (elt vec i))
                     el)
                   elts-arr))
    (setf (pb/raw-vector/elements raw) elts-arr)
    raw))

;;; -------------------------------------------------------------------------

(defun pb/reset-reaction (cs-reaction-msg obj-state-msg)
  (setf (pb/msg/description cs-reaction-msg) (pb/msg/description obj-state-msg)
     (pb/msg/channel-id cs-reaction-msg) (pb/msg/channel-id obj-state-msg)
     (pb/msg/direction cs-reaction-msg) +pb/direction/from-cs-to-obj+
     (pb/msg/age cs-reaction-msg) (1+ (pb/msg/age obj-state-msg)))
  
  (when (or (null (pb/msg/components cs-reaction-msg))
           (/= (length (pb/msg/components cs-reaction-msg))
              (length (pb/msg/components obj-state-msg))))
    (setf (pb/msg/components cs-reaction-msg)
       (pb/new-object-component-array
        (length (pb/msg/components obj-state-msg)))))
  
  (loop
     for obj-component/state across (pb/msg/components obj-state-msg)
     for obj-component/reaction across (pb/msg/components cs-reaction-msg)
     do
       (progn
         (setf (pb/obj-component/description obj-component/reaction)
            (pb/obj-component/description obj-component/state)
            (pb/obj-component/prefix-name obj-component/reaction)
            (pb/obj-component/prefix-name obj-component/state)
            (pb/obj-component/name obj-component/reaction)
            (pb/obj-component/name obj-component/state))
         
         (cond
           ((pb/obj-component/has-raw-vector obj-component/state)
            (pb/obj-component/raw-vector obj-component/reaction))
           (t
            (dolist (resetter *pb/obj-component-reaction-resetters-alist*)
              (funcall (cdr resetter)
                       obj-component/state obj-component/reaction))))))
  t)

;;; -------------------------------------------------------------------------

(defparameter *control-object-state* nil)
(defparameter *control-system-reaction* nil)

(defun reset-control-system ()
  (setf *control-object-state* (pb/new-message)
        *control-system-reaction* (pb/new-message)))

(defun deserialize-control-object-state (buffer)
  (pb:clear *control-object-state*)
  (pb:merge-from-array *control-object-state* buffer 0 (length buffer))
  (if (eql *control-system-channel-id*
           (pb/msg/channel-id *control-object-state*))
      *control-object-state*
      nil))

(defun calculate-control-system-reaction (obj-state-msg)
  (pb/reset-reaction *control-system-reaction* obj-state-msg)
  (funcall *control-system-function*
           obj-state-msg *control-system-reaction*))

(defun serialize-control-system-reaction (cs-reaction-msg)
  (let* ((size (pb:octet-size cs-reaction-msg))
         (buffer (make-array size :element-type '(unsigned-byte 8))))
    (pb:serialize cs-reaction-msg buffer 0 size)
    buffer))
