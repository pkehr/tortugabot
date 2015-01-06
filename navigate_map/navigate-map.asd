
(in-package :cl-user)

(asdf:defsystem navigate-map
  :name "navigate-map"
  :author "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :maintainer "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :version "0.1"
  :licence "BSD"
  :description "Using move_base from Lisp"

  :depends-on (roslisp actionlib actionlib_msgs-msg move_base_msgs-msg cl-tf)

  :components
  ((:module "src"
	    :components
	    ((:file "package")
	     (:file "simple-nav-goals" :depends-on ("package"))))))
