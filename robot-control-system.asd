;;;; trunk-robot-control-system.asd
;;;;
;;;; Copyright (c) 2017 Ivan Podmazov (podmazov@gmail.com)

(asdf:defsystem #:robot-control-system
  :description "Common Lisp implementation of the control system of the Simulink mechanics model"
  :author "Ivan Podmazov (podmazov@gmail.com)"
  :license "GPLv3"
  :defsystem-depends-on (#:protobuf)
  :depends-on (#:alexandria
               #:protobuf
               #:websocket-driver)
  :serial t
  :components
  ((:static-file "README.md")
   (:protobuf-source-file
    "simulink_mechanics"
    :proto-pathname "protos/components/simulink_mechanics/simulink_mechanics")
   (:protobuf-source-file
    "control_object_component"
    :proto-pathname "protos/control_object_component"
    :depends-on ("simulink_mechanics")
    :proto-search-path
    ("protos/components/simulink_mechanics/"))
   (:protobuf-source-file
    "control_system_communication_protocol"
    :proto-pathname "protos/control_system_communication_protocol"
    :depends-on ("control_object_component")
    :proto-search-path
    ("protos/"
     "protos/components/simulink_mechanics/"))
   
   (:file "package" :depends-on ("control_system_communication_protocol"))
   (:file "protocol-buffers" :depends-on ("package"))
   (:static-file "server-address.sexp")
   (:file "io" :depends-on ("protocol-buffers"))
   
   (:file "coordinates" :depends-on ("package"))
   (:file "reference-frame" :depends-on ("coordinates"))
   (:file "pid-controller" :depends-on ("package"))
   (:file "fsm" :depends-on ("package"))
   (:file "control-scenario" :depends-on ("fsm"))
   
   (:file "simulink-mechanics" :depends-on ("protocol-buffers"))
   
   (:file "control-objects/models/spherical-robot/spherical-robot"
          :depends-on ("coordinates"
                       "reference-frame"
                       "pid-controller"
                       "control-scenario"
                       "simulink-mechanics"))
   (:file "control-objects/models/spherical-robot/movements"
          :depends-on ("control-objects/models/spherical-robot/spherical-robot"))))
