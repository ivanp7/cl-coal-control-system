;;;; package.lisp
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(defpackage #:robot-control-system
  (:use #:cl)
  (:export :*server-address*
           :is-online-p
           :connect
           :disconnect
           :reconnect))
