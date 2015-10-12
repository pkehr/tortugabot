(in-package :find-boards)

(defparameter *result* (make-hash-table :test 'equal))

(defparameter *message* nil)

(defun msg->transform-stamped (msg)
  (with-fields ((frame-id (frame_id header))
                (stamp (stamp header))
                (x (x translation transform))
                (y (y translation transform))
                (z (z translation transform))
                (ax (x rotation transform))
                (ay (y rotation transform))
                (az (z rotation transform))
                (aw (w rotation transform)))
      msg
    (cl-tf:make-pose-stamped
     frame-id 0
     (make-3d-vector x y z)
     (make-quaternion ax ay az aw))))

(defparameter *transform-listener* (make-instance 'cl-tf:transform-listener))

(defun our-transform (trans)
  (cl-tf:transform-pose *transform-listener* :pose trans :target-frame "/map"))

(defun save-board-pos (msg)
  (setf *message* msg)
  (unless (gethash (geometry_msgs-msg:child_frame_id msg) *result*)
    (setf (gethash (geometry_msgs-msg:child_frame_id msg) *result*) 
        (our-transform (msg->transform-stamped msg)))))

(defun find-boards()
  (with-ros-node ("board-pos-listener" :spin t)
    (subscribe "/ar_multi_boards/transform" "geometry_msgs/TransformStamped" #'save-board-pos))) 
