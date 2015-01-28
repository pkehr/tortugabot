(asdf:defsystem sound-play-lisp
  :name "sound-play-lisp"
  :author "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :version "0.0.1"
  :maintainer "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :licence "BSD"
  :description "sound play lisp action client"
  :depends-on (roslisp sound_play-msg actionlib actionlib_msgs-msg)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "guts" :depends-on ("package"))))))
