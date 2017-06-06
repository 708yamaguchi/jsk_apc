#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-package "force_proximity_ros")
(ros::load-ros-package "jsk_recognition_msgs")

;; initialize pose of the real fetch robot
(defun init()
  (send *fetch* :reset-pose)
  (send *fetch* :head :neck-p :joint-angle 20) ;; see down
  (send *ri* :angle-vector (send *fetch* :angle-vector) 10000)
  (send *ri* :stop-grasp)
  (send *ri* :wait-interpolation))


;; order the real fetch robot to follow the joint angle of the model robot
(defun move (time &optional (trial-number 100))
  (let ((test nil) (i 0))
    (while (and (null test) (< i trial-number))
      (setq i (+ i 1))
      (setq test (send *ri* :angle-vector (send *fetch* :angle-vector) time)))
    (send *ri* :wait-interpolation)))


(defun set-arm (&key (time 5000) (x 600) (y 0) (z 770) (from-top t))
  (let ((coords))
    (if from-top
	;;from top
	(progn
	  (setq coords (make-coords :pos (float-vector x y z) :rpy (float-vector 0 pi/2 0)))
	  (send *fetch* :inverse-kinematics coords)
	  (move time))
      ;;from front
      (progn
	(setq coords (make-coords :pos (float-vector x y z)))
	(send *fetch* :inverse-kinematics coords)
	(move time)))
    (send *ri* :stop-grasp)
    coords))



(defun cb-proximity (proximity-arr)
    (setq proximity0-obj (elt (send proximity-arr :proximities) 0))
    (setq proximity1-obj (elt (send proximity-arr :proximities) 1))
    (setq proximity2-obj (elt (send proximity-arr :proximities) 2))
    (setq proximity3-obj (elt (send proximity-arr :proximities) 3))
    (setq proximity4-obj (elt (send proximity-arr :proximities) 4)))


(defun echo-proximity()
    (ros::subscribe
     "/gripper_front/limb/right/proximity_array"
     force_proximity_ros::ProximityArray #'cb-proximity 1))


(defun FA-I-calibration (&optional (count 10))
  (let ((prox0-value-sum 0)
	(prox1-value-sum 0)
	(prox2-value-sum 0)
	(prox3-value-sum 0)
	(prox4-value-sum 0)
	(co 1.5))

    (dotimes (i count)
      (ros::spin-once)
      (setq prox0-value-sum (+ prox0-value-sum (send proximity0-obj :average)))
      (setq prox1-value-sum (+ prox1-value-sum (send proximity1-obj :average)))
      (setq prox2-value-sum (+ prox2-value-sum (send proximity2-obj :average)))
      (setq prox3-value-sum (+ prox3-value-sum (send proximity3-obj :average)))
      (setq prox4-value-sum (+ prox4-value-sum (send proximity4-obj :average))))

    (setq proximity0-init-value (/ prox0-value-sum count))
    (setq proximity1-init-value (/ prox1-value-sum count))
    (setq proximity2-init-value (/ prox2-value-sum count))
    (setq proximity3-init-value (/ prox3-value-sum count))
    (setq proximity4-init-value (/ prox4-value-sum count))

    (setq k0 (expt (/ proximity0-init-value 10000.0) co))
    (setq k1 (expt (/ proximity1-init-value 10000.0) co))
    (setq k2 (expt (/ proximity2-init-value 10000.0) co))
    (setq k3 (expt (/ proximity3-init-value 10000.0) co))
    (setq k4 (expt (/ proximity4-init-value 10000.0) co))))

;; move arm using proximity sensor 3,4
;; p feedback (pid)
;; use in find-surface-from-top ?
;; DO NOT forget to fa-i-calibration & start-grasp before use this function

;; WIP ;;
(defun proximity-correction(&optional (count 1))
  (let ((proximity-diff) (proximity1) (proximity2)
	(end-coords) (p 0.01))
    (dotimes (i count)
      (ros::spin-once)
      (setq proximity1 (- (send proximity1-obj :average) proximity1-init-value))
      (setq proximity2 (- (send proximity2-obj :average) proximity2-init-value))
      (setq proximity-diff (- proximity1 proximity2))
      (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
      (send end-coords :translate (float-vector 0 (* p proximity-diff) 0))
      (send *fetch* :inverse-kinematics end-coords)
      (move 500) ;; (move) includes :wait-interpolation
    )))


;; detect plane pose and grasp
;; use only proximity3 & proximity4 sensor
(defun measure-plane-pose()
  ;;(FA-I-calibration)
  ;;(send *co* :clear-all)
  ;;(table-to-moveit) ;; send table to moveit as collision object
  (let* ((proximity3 (calc-prox 3))
	 (proximity4 (calc-prox 4))
	(proximity-diff) (end-coords) (rgripper-coords) (lgripper-coords)
	(obj-top-height) (arm-height)
	(threshold1 2000) (threshold2 200) (time-obj (ros::time-now) (start-time))
	(av) (avs nil) (devide 50) (time-list (make-list devide :initial-element (/ 5000 devide))))
    (send *ri* :start-grasp)

    ;; move fetch arm toward object
    (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
    (send end-coords :translate #f(200 0 0))
    (send *ri* :angle-vector-raw (send *fetch* :inverse-kinematics end-coords) 7000)
    (setq start-time (progn (send time-obj :now) (send time-obj :sec)))
    (while (and (< proximity3 threshold1)
		(< proximity4 threshold1)
		(< (- (progn (send time-obj :now) (send time-obj :sec)) 11) start-time))
      (ros::spin-once)
      (setq proximity3 (calc-prox 3))
      (setq proximity4 (calc-prox 4))
      (format t "fprox3 : ~A~%" proximity3)
      (format t "fprox4 : ~A~%" proximity4)
      )
    (send *ri* :cancel-angle-vector)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))

    (format t "stop forwarding~%")

    ;; rotate fetch arm toword object
    (setq proximity-diff (- proximity3 proximity4))
    (if (< 0 proximity-diff)
	(progn

	  ;; TODO ;;
	  ;; change rotate center from gripper-coords to finger tip which is nearer to object than another finger tip.
	  ;; rgripper-coords done ??

	  (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
	  (setq rgripper-coords (make-cascoords :coords end-coords)) ;;calc rotation center
	  (send rgripper-coords :translate #f(40 10 0))
	  (setq end-coords (make-cascoords :coords end-coords))
	  (send rgripper-coords :assoc end-coords) ;; parent : gripper-coords, child : end-coords
	  (dotimes (i devide)
	    (send rgripper-coords :rotate (/ (/ pi 4) devide) :z)
        (print rgripper-coords)
        (print end-coords)

        (print i)

	    (setq av (send *fetch* :inverse-kinematics end-coords))
	    (setq avs (cons av avs)))

	  (setq avs (reverse avs))
	  (send *ri* :angle-vector-sequence avs time-list)

	  (objects (list *fetch* rgripper-coords end-coords))

	  (setq start-time (progn (send time-obj :now) (send time-obj :sec)))
	  (while (and (< 0 proximity-diff)
		      (< (- (progn (send time-obj :now) (send time-obj :sec)) 6) start-time))
	    (ros::spin-once)
	    (setq proximity3 (calc-prox 3))
	    (setq proximity4 (calc-prox 4))
	    (setq proximity-diff (- proximity3 proximity4))
	    (format t "1prox3 : ~A~%" proximity3)
	    (format t "1prox4 : ~A~%" proximity4)
	    ))
      (progn

	;; TODO ;;
	;; change rotate center from gripper-coords to finger tip which is nearer to object than another finger tip.

	(setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
	(setq lgripper-coords (make-cascoords :coords end-coords)) ;;calc rotation center
	(send lgripper-coords :translate #f(40 -10 0))
	(setq end-coords (make-cascoords :coords end-coords))
	(send lgripper-coords :assoc end-coords) ;; parent : gripper-coords, child : end-coords

	(dotimes (i devide)
	  (send lgripper-coords :rotate (/ (/ -pi 4) devide) :z)

      ;;(objects (list *fetch* rgripper-coords end-coords))
      (print i)

	  (setq av (send *fetch* :inverse-kinematics end-coords))
	  (setq avs (cons av avs)))

	(setq avs (reverse avs))
	(send *ri* :angle-vector-sequence avs time-list)

	(objects (list *fetch* lgripper-coords end-coords))

	(setq start-time (progn (send time-obj :now) (send time-obj :sec)))
	(while (and (> 0 proximity-diff)
		    (< (- (progn (send time-obj :now) (send time-obj :sec)) 6) start-time))
	  (ros::spin-once)
	  (setq proximity3 (calc-prox 3))
	  (setq proximity4 (calc-prox 4))
	  (setq proximity-diff (- proximity3 proximity4))
	  (format t "prox3 : ~A~%" proximity3)
	  (format t "prox4 : ~A~%" proximity4)
	  )))
    (send *ri* :cancel-angle-vector)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))


    (send *ri* :stop-grasp)

#|
    ;; correct hand position (y axis)
    (ros::spin-once)
    (setq proximity3 (calc-prox 3))
    (setq proximity4 (calc-prox 4))
    (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
    (format t "start correcting hand pos~%")

    (if (> proximity3 proximity4)
	(progn
	  (send end-coords :translate #f(0 50 0))
	  (format t "translate1~%")
	  (send *ri* :angle-vector (send *fetch* :inverse-kinematics end-coords) 5000)
	  (setq start-time (progn (send time-obj :now) (send time-obj :sec)))
	  (while (and (> proximity3 proximity4)
		      (< (- (progn (send time-obj :now) (send time-obj :sec)) 6) start-time))
	    (ros::spin-once)
	    (setq proximity3 (calc-prox 3))
	    (setq proximity4 (calc-prox 4))
	    (format t "$prox3 : ~A~%" proximity3)
	    (format t "$prox4 : ~A~%" proximity4)))
      (progn
	(send end-coords :translate #f(0 -50 0))
	(format t "translate2~%")
	(send *ri* :angle-vector (send *fetch* :inverse-kinematics end-coords) 5000)
	(setq start-time (progn (send time-obj :now) (send time-obj :sec)))
	(while (and (> proximity3 proximity4)
		    (< (- (progn (send time-obj :now) (send time-obj :sec)) 2) start-time))
	  (ros::spin-once)
	  (setq proximity3 (calc-prox 3))
	  (setq proximity4 (calc-prox 4))
	  (format t "(prox3 : ~A~%" proximity3)
	  (format t "(prox4 : ~A~%" proximity4))))
    (send *ri* :cancel-angle-vector)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
|#

    ;; move fetch arm forward and grasp
    (format t "I am going to grasp.~%")
    (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
    (send end-coords :translate #f(60 0 0))
    (send *ri* :angle-vector-raw (send *fetch* :inverse-kinematics end-coords) 2000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp)
    ;;(send *ri* :angle-vector (send *fetch* :reset-pose) 10000)
    ;;(send *ri* :wait-interpolation)
    ))


;; wip
#|
    ;; move fetch arm up
    (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
    (send end-coords :translate #f(0 0 200))
    (send *ri* :angle-vector (send *fetch* :inverse-kinematics end-coords) 3000)
    (setq start-time (progn (send time-obj :now) (send time-obj :sec)))
    (while (and (< threshold2 proximity3)
		(< threshold2 proximity4)
		(< (- (progn (send time-obj :now) (send time-obj :sec)) 4) start-time))
      (ros::spin-once)
      (setq proximity3 (- (send proximity3-obj :average) proximity3-init-value))
      (setq proximity4 (- (send proximity4-obj :average) proximity4-init-value))
      (format t "prox3 :: ~A~%" proximity3)
      (format t "prox4 :: ~A~%" proximity4))
    (send *ri* :cancel-angle-vector)
    (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))
    (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
    (setq obj-top-height (elt (send end-coords :worldpos) 2))
    (send end-coords :translate #f(0 0 50))
    (send *ri* :angle-vector (send *fetch* :inverse-kinematics end-coords) 1000)

    ;; send collision object to moveit
    (format t "make collision object~%")
    (send *co*
	  :add-object (make-cube 250 500 (+ obj-top-height 50))
	  :frame-id "base_link"
	  :object-id "target-obj"
	  :relative-pose (send (send end-coords :copy-coords)
			       :translate (float-vector 150 0 (/ (+ obj-top-height 50) -2))))
    ;;(unix:sleep 1)

    ;; rotate fetch arm
    (setq object-coords (make-cascoords :coords end-coords)) ;;calc rotation center
    (send object-coords :translate #f(30 0 0))
    (setq end-coords (make-cascoords :coords end-coords))
    (send object-coords :assoc end-coords) ;; parent : gripper-coords, child : end-coords
    (send object-coords :rotate pi/2 :y)
    (send object-coords :rotate pi/2 :x)
    (send end-coords :translate #f(0 30 0)) ;; 30 : offset
    (send *fetch* :inverse-kinematics end-coords)

    (move 3000) ;; (move) includes :wait-interpolation

    ;; delete collision object from moveit
    (send *co* :clear-all)

    ;; correct hand position (y axis)
    (send *ri* :stop-grasp)
    (ros::spin-once)
    (setq proximity3 (- (send proximity3-obj :average) proximity3-init-value))
    (setq proximity4 (- (send proximity4-obj :average) proximity4-init-value))
    (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
    (format t "start correct hand pos~%")
    (if (> proximity3 proximity4)
	(progn
	  (send end-coords :translate #f(0 50 0))
	  (send *ri* :angle-vector (send *fetch* :inverse-kinematics end-coords) 1000)
	  (setq start-time (progn (send time-obj :now) (send time-obj :sec)))
	  (while (and (> proximity3 proximity4)
		      (< (- (progn (send time-obj :now) (send time-obj :sec)) 2) start-time))
	    (ros::spin-once)
	    (setq proximity3 (- (send proximity3-obj :average) proximity3-init-value))
	    (setq proximity4 (- (send proximity4-obj :average) proximity4-init-value))))
      (progn
	(send end-coords :translate #f(0 -50 0))
	(send *ri* :angle-vector (send *fetch* :inverse-kinematics end-coords) 1000)
	(setq start-time (progn (send time-obj :now) (send time-obj :sec)))
	(while (and (> proximity3 proximity4)
		    (< (- (progn (send time-obj :now) (send time-obj :sec)) 2) start-time))
	  (ros::spin-once)
	  (setq proximity3 (- (send proximity3-obj :average) proximity3-init-value))
	  (setq proximity4 (- (send proximity4-obj :average) proximity4-init-value)))))
      (send *ri* :cancel-angle-vector)
      (send *fetch* :angle-vector (send *ri* :state :potentio-vector :wait-until-update t))


    ;; lower fetch arm and grasp
    (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
    (setq arm-height (elt (send end-coords :worldpos) 2))
    (send end-coords :translate (float-vector (+ (- arm-height obj-top-height) 30) 0 0)) ;;gripper offset : 30
    (send *ri* :angle-vector (send *fetch* :inverse-kinematics end-coords) 3000)
    (send *ri* :wait-interpolation)
    (send *ri* :start-grasp)))
|#


;; compared to (measure-plane-pose)
(defun demo-compare ()
  (send *ri* :stop-grasp)

  ;; move fetch arm toward object
  (setq end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords))
  (send end-coords :translate #f(90 0 0))
  (send *ri* :angle-vector-raw (send *fetch* :inverse-kinematics end-coords) 5000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp))



;; WIP
(defun grasp-object-in-cupboard ()

  ;;memo
  ;;(progn (setq obj-array (instance jsk_recognition_msgs::BoundingBoxArray)) (while (null (send obj-array :boxes)) (setq obj-array (one-shot-subscribe "/HSI_color_filter/boxes" jsk_recognition_msgs::BoundingBoxArray))))


  ;; get object-coords using hsi color filter
  (let* ((obj-array (one-shot-subscribe
			"/HSI_color_filter/boxes"
			jsk_recognition_msgs::BoundingBoxArray))
	 (obj-coords (send (car obj-array) :worldcoords))
	 (obj-x (elt (send obj-coords :worldpos) 0))
	 (obj-y (elt (send obj-coords :worldpos) 1))
	 (obj-z (elt (send obj-coords :worldpos) 2))
	 (target-direction (atan (/ obj-y obj-x))))
    (send *ri* :angle-vector
	  (send *fetch* :inverse-kinematics
		(send (send (send obj-coords :worldcoords) :copy-coords)
		      :locate (float-vector 700 obj-y obj-z))) 5000)
    (FA-I-calibration)
    (send *ri*
	  :move-to (make-coords :pos (float-vector obj-x obj-y 0)
				:rpy (float-vector 0 0 target-direction))
	  :frame-id "/base_link")

    ;; move arm forward and grasp object
    (measure-plane-pose)
    ))


(defun find-surface-from-top(coords)
  ;before use this function, do below
  ;(send *ri* :angle-vector (send *fetch* :inverse-kinematics (make-coords :pos #f(800 0 810))))

  (send *fetch* :inverse-kinematics coords)
  (move 2000) ;;(move) includes :wait-interpolation
  (send *ri* :stop-grasp)

  (let ((proximity0-prev 0) (proximity0 0)
	(proximity3-prev 0) (proximity3 0)
	(proximity4-prev 0) (proximity4 0)
        (i 0) (threshold 50)
	;;which sensor touch object (sensor0, 3, 4)
	(return-list '(t t t)))

    (format t "(t t t)1 : ~A~%" return-list)
    (setq return-list '(t t t))
    (format t "(t t t)2 : ~A~%" return-list)

    (unix:sleep 1)
    (FA-I-calibration)

    (while (and (< (- proximity0 proximity0-prev) threshold)
		(< (- proximity3 proximity3-prev) threshold)
		(< (- proximity4 proximity4-prev) threshold))
      (setq i (+ i 5))

      (send *fetch* :inverse-kinematics (make-coords :pos (v- (send coords :worldpos) (float-vector 0 0 i)) :rpy (car (send (send coords :worldcoords) :rpy-angle))))
      ;;(send *fetch* :torso :waist-z :joint-angle (- (send *fetch* :torso :waist-z :joint-angle) 10))
      (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 500)
      (send *ri* :wait-interpolation)

      (ros::spin-once)

      (setq proximity0-prev proximity0)
      (setq proximity3-prev proximity3)
      (setq proximity4-prev proximity4)
      (setq proximity0 (- (send proximity0-obj :average) proximity0-init-value))
      (setq proximity3 (- (send proximity3-obj :average) proximity3-init-value))
      (setq proximity4 (- (send proximity4-obj :average) proximity4-init-value))

      (format t "i : ~A~%" i)
      (format t "proximity0-diff : ~A~%" (- proximity0 proximity0-prev))
      (format t "proximity3-diff : ~A~%" (- proximity3 proximity3-prev))
      (format t "proximity4-diff : ~A~%" (- proximity4 proximity4-prev)))

    (format t "find surface!~%")

    (format t "(t t t)3 : ~A~%" return-list)
    (setq return-list '(t t t))
    (format t "(t t t)4 : ~A~%" return-list)
    (setq return-list '(t t t))

    ;; ERROR ;;
    (format t "(t t t)5 : ~A~%" return-list)
    ;; ERROR ;;

    (setf (elt return-list 0) (if (< (- proximity0 proximity0-prev) threshold) nil t))
    (setf (elt return-list 1) (if (< (- proximity3 proximity3-prev) threshold) nil t))
    (setf (elt return-list 2) (if (< (- proximity4 proximity4-prev) threshold) nil t))

    (format t "sensor-list : ~A~%" return-list)
    return-list))


(defun grasp(coords)
  ;; grasp with sensor1,2 (feedback grope-grasp)
  ;; First, try to grasp. Next, if fail, call grope again

  (send *fetch* :torso :waist-z :joint-angle (+ (send *fetch* :torso :waist-z :joint-angle) 15))
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (let ((fail nil) (next-coords))
    ;; TODO : CHANGE CONDITION(?)
    (if (< (send (send *ri* :start-grasp) :position) 0.02)
    ;; TODO END ;;
	(setq fail t))

    (if fail
	(progn
	  (format t "grasp failed, try grope again.~%")
	  (setq next-coords (decide-next-coords coords))
	  (format t "next-coords : ~A~%" next-coords)
	  (grope next-coords))
      (format t "trial succeeded!~%"))))


;; (arg) coords -> (return) next-coords
(defun decide-next-coords(coords)
  ;; decide where to move arm
  (let ((co (send (send coords :worldcoords) :copy-coords)) (end-coords (send (send (send *fetch* :rarm-end-coords) :worldcoords) :copy-coords)))
	(mid-coords) (mid-pos))
    (cond
     ((eq *mode* 1) co)
     ((eq *mode* 2) (setq *rprev-coords* nil) (setq *lprev-coords* nil) (send co :rotate (/ pi 6) :x))
     ((eq *mode* 3)
      (setq *rprev-coords* coords)
      (if (null *lprev-coords*)
	  ;;move arm in order to touch with left finger
	  (send co :translate #f(0 -120 0))
	;;move arm to midpoint between now-coords and *lprev-coords*
	(progn
	  (format t "boundp *lprev-coords*~%")
	  ;;(setq mid-pos (scale 0.5 (v+ (send co :worldpos) (send *lprev-coords* :worldpos))))
	  (setq mid-pos (midpoint 0.5 (send co :worldpos) (send *lprev-coords* :worldpos)))
	  (setq mid-coords (make-coords :pos mid-pos :rpy (car (send (send co :worldcoords) :rpy-angle)))))))
     ((eq *mode* 4)
      (setq *lprev-coords* coords)
      (if (null *rprev-coords*)
	  ;;move arm in order to touch with right finger
	  (send co :translate #f(0 120 0))
	;;move arm to midpoint between now-coords and *rprev-coords*
	(progn
	  (format t "boundp *rprev-coords*~%")
	  ;;(setq mid-pos (scale 0.5 (v+ (send co :worldpos) (send *rprev-coords* :worldpos))))
	  (setq mid-pos (midpoint 0.5 (send co :worldpos) (send *rprev-coords* :worldpos)))
	  (setq mid-coords (make-coords :pos mid-pos :rpy (car (send (send co :worldcoords) :rpy-angle)))))))
     (t (format t "error~%")))))


;; global variable
(setq *mode* 0) ;; 1 : paml touch, 2 : both finger touch, 3 : right finger touch, 4 : left finger touch
(setq *rprev-coords* nil) ;; last coords which touch object with only right finger
(setq *lprev-coords* nil) ;; last coords which touch object with only left finger
;; global variable


;; coords : initial coordinates of gripper
(defun grope (coords)
  (setq *mode* 0)
  ;; TODO : CHANGE CONDITION, ex (< (aref (send coords :worldrot) 0) pi)
  (if (< (elt (car (send (send coords :worldcoords) :rpy-angle)) 2) pi)
  ;; TODO END ;;
      (progn
	(format t "coords : ~A~%" coords)
	(let ((sensor-list '(nil nil nil)))
	  (setq sensor-list '(nil nil nil))
	  (setq sensor-list (find-surface-from-top coords))

	  (if (elt sensor-list 0)
	      (progn
		(setq *mode* 1)
		(format t "*mode* : ~A~%" *mode*)
		(grasp coords)) ;; palm touch with sensor
	    (if (and (elt sensor-list 1) (elt sensor-list 2))
		(progn
		  (setq *mode* 2)
		  (format t "*mode* : ~A~%" *mode*)
		  (grasp coords)) ;; both finger touch
	      (if (and (null (elt sensor-list 1)) (elt sensor-list 2))
		  (progn
		    (setq *mode* 3)
		    (format t "*mode* : ~A~%" *mode*)
		    (grasp coords)) ;; right finger touch
		(progn
		  (setq *mode* 4)
		  (format t "*mode* : ~A~%" *mode*)
		  (grasp coords))))))) ;; left finger touch

    (format t "trial failed~%")))


(defun table-to-moveit ()
  (send *co*
	:add-object (make-cube 600 800 730)
	:frame-id "base_link"
	:relative-pose (make-coords :pos #f(1000 0 365))))


(defun get-calibrated-value ()
  (FA-I-calibration)
  (let ((proximity0) (proximity1) (proximity2) (proximity3) (proximity4))
    (while t
      (ros::spin-once)
      (setq proximity0 (calc-prox 0))
      (setq proximity1 (calc-prox 1))
      (setq proximity2 (calc-prox 2))
      (setq proximity3 (calc-prox 3))
      (setq proximity4 (calc-prox 4))
      (format t "~0,5F ~0,5F ~0,5F ~0,5F ~0,5F~%" proximity0 proximity1 proximity2 proximity3 proximity4))))


(defun calc-prox (number)
  (cond ((eq number 0) (/ (- (send proximity0-obj :average) proximity0-init-value) k0))
	((eq number 1) (/ (- (send proximity1-obj :average) proximity1-init-value) k1))
	((eq number 2) (/ (- (send proximity2-obj :average) proximity2-init-value) k2))
	((eq number 3) (/ (- (send proximity3-obj :average) proximity3-init-value) k3))
	((eq number 4) (/ (- (send proximity4-obj :average) proximity4-init-value) k4))
	(t nil)))


;; init
(defun start ()
  (require "package://fetcheus/fetch-interface.l")
  (fetch-init)
  (format t "done fetch-init~%")
  (echo-proximity)
  (unix:sleep 1)
  (ros::spin-once)
  (unix:sleep 1)
  (FA-I-calibration)
  (format t "done FA-I sensor calibration~%")
  (load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
  ;;(setq *co* (instance collision-object-publisher :init))
  ;;(format t "done collision-object-publisher init~%")
)
