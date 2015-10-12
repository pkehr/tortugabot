
(in-package :cl-user)

(asdf:defsystem find_boards
  :name "find-boards"
  :author "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :maintainer "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :version "0.1"
  :licence "BSD"
  :description "Find boards"

  :depends-on (roslisp geometry_msgs-msg move_base_msgs-msg cl-tf)

  :components
  ((:module "src"
	    :components
	    ((:file "package")
	     (:file "find-boards" :depends-on ("package"))))))
