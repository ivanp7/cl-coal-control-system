;;;; io.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(in-package #:robot-control-system)

(defparameter *server-address*
  (with-open-file (file (asdf:system-relative-pathname
                         :robot-control-system "server-address.sexp"))
    (read file)))

(defvar *client-websocket* nil)

(defun is-online-p ()
  (not (null *client-websocket*)))

(defun connect (&key (server-ip (first *server-address*))
                  (server-port (second *server-address*)))
  (unless (is-online-p)
    (not (null
        (progn
          (reset-control-system)
          (setf *client-websocket*
             (wsd:start-connection
              (let ((client (wsd:make-client
                             (format nil "ws://~A:~A/" server-ip server-port))))
                (wsd:on :message client
                        (lambda (buffer)
                          (let ((message (deserialize-control-object-state
                                          buffer)))
                            (when message
                              (wsd:send-binary
                               client
                               (serialize-control-system-reaction
                                (calculate-control-system-reaction
                                 message)))))))
                client))))))))

(defun disconnect ()
  (when (is-online-p)
    (prog1 (wsd:close-connection *client-websocket*)
      (setf *client-websocket* nil))))

(defun reconnect (&key (server-ip (first *server-address*))
                    (server-port (second *server-address*)))
  (disconnect)
  (connect :server-ip server-ip :server-port server-port))
