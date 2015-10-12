
(in-package :cl-user)

(asdf:defsystem travel_hall
  :name "travel_hall"
  :author "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :maintainer "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :version "0.1"
  :licence "BSD"
  :description "Using move_base from Lisp"

  :depends-on (navigate-map roslisp actionlib actionlib_msgs-msg move_base_msgs-msg cl-tf)

  :components
  ((:module "src"
	    :components
	    ((:file "package")
	     (:file "travel_hall" :depends-on ("package"))))))
