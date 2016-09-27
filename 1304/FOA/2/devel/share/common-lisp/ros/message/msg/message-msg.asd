
(cl:in-package :asdf)

(defsystem "message-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "location" :depends-on ("_package_location"))
    (:file "_package_location" :depends-on ("_package"))
  ))