(in-package :travel-hall)
(defvar *move-base-client* nil)

(defun init-action-client ()
  (setf *move-base-client* (actionlib:make-action-client
                            "move_base"
                            "move_base_msgs/MoveBaseAction"))
  (roslisp:ros-info (navigate-map)
                    "Waiting for move_base action server...")
  ;; workaround for race condition in actionlib wait-for server
  (loop until (actionlib:wait-for-server *move-base-client*))
  (roslisp:ros-info (navigate-map) 
                    "move_base action client created."))

(defun get-action-client ()
  (when (null *move-base-client*)
    (init-action-client))
  *move-base-client*)

(defun make-move-base-goal (pose-stamped-goal)
  (actionlib:make-action-goal (get-action-client)
    target_pose pose-stamped-goal))

(defun call-move-base-action (frame-id translation rotation)
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "move-base-lisp-client"))

  (multiple-value-bind (result status)
      (let ((actionlib:*action-server-timeout* 10.0)
            (the-goal (cl-tf:pose-stamped->msg 
                       (cl-tf:make-pose-stamped
                        frame-id
                        (roslisp::ros-time)
                        translation rotation))))
        (actionlib:call-goal
         (get-action-client)
         (make-move-base-goal the-goal)))
    (roslisp:ros-info (navigate-map) "Move_base action finished.")
    (values result status)))

(defun travel ()
  ;; 1. Tür rechts
  (print "door 1 right")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 23.341 36.574 0)(cl-transforms:make-quaternion 0 0 0.562 0.827))
  (sleep 1)
  ;; 2. Tür rechts
  (print "door 2 right")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 20.203 37.997 0)(cl-transforms:make-quaternion 0 0 0.577 0.816))
  (sleep 1)
  ;; 1. Tür links
  (print "door 1 left")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 19.519 38.078 0)(cl-transforms:make-quaternion 0 0 0.876 -0.482))
  (sleep 1)  
  ;; 2. Tür links
  (print "door 2 left")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 18.694 38.536 0)(cl-transforms:make-quaternion 0 0 0.876 -0.482))
  (sleep 1)  
  ;; 3. Tür rechts
  (print "door 3 right")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 17.488 39.084 0)(cl-transforms:make-quaternion 0 0 0.541 0.841))
  (sleep 1)  
  ;; 3. Tür links
  (print "door 3 left")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 17.230 39.234 0)(cl-transforms:make-quaternion 0 0 -0.857 0.516))
  (sleep 1)  
  ;; 4. Tür rechts
  (print "door 4 right")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 13.343 41.424 0)(cl-transforms:make-quaternion 0 0 0.457 0.889))
  (sleep 1)  
  ;; 4. Tür links
  (print "door 4 left")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 12.762 41.902 0)(cl-transforms:make-quaternion 0 0  0.867 -0.498))
  (sleep 1)  
  ;; 5. Tür rechts
  (print "door 5 right")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 9.823 43.6767 0)(cl-transforms:make-quaternion 0 0 0.433 0.901))
  (sleep 1)  
  ;; 5. Tür links
  (print "door 5 left")
  (call-move-base-action "map" (cl-transforms:make-3d-vector 9.635 43.856 0)(cl-transforms:make-quaternion 0 0 0.893 -0.450))
  (sleep 1)  
)
