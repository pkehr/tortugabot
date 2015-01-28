
(in-package :sound-play-lisp)

(defun say (a-string)
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "sound-play-node"))
  (let ((publisher (roslisp:advertise "robotsound" 'sound_play-msg:<soundrequest>)))
    (loop while (< (roslisp:num-subscribers publisher) 1) do (sleep 0.01))
    (ros-info (sound-play) "saying ~a" a-string)
    (roslisp:publish-msg
     publisher
     :sound (symbol-code 'sound_play-msg:<soundrequest> :say)
     :command (symbol-code 'sound_play-msg:<soundrequest> :play_once)
     :arg a-string :arg2 "voice_kal_diphone")))
