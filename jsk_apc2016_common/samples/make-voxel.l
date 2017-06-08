#!/usr/bin/env roseus

;; (objects (make-voxel-list 5 5 5 100))
(defun make-voxel-list (&optional (x 5) (y 5) (z 5) (size 100))
  (let ((voxel-list nil) (cube) (location) (show-size (* 1 size))) ;;show-size 0.99 or 1
    (dotimes (i (* x y z))
      (setq location `(,(mod i x) ,(/ (mod i (* x y)) x) ,(/ i (* x y))))
;;      (setq voxel-list (append (list (make-cube size size size :set-color :blue :name `(,(mod i x) ,(floor (/ i x)) ,(floor (/ i (* x y))))) voxel-list)))
      (setq cube (make-cube show-size show-size show-size :name location))
;;      (send cube :locate (scale size (float-vector (elt location 0) (elt location 1) (elt location 2))))
      (send cube :locate (scale size (list2fv location)))
      (send cube :set-color :green)
      (setq voxel-list (append voxel-list (list cube))))
    voxel-list
    ))


(defun move-voxel-list (voxel-list pos) ;; ex. pos #f(100 100 100)
  (let ((voxel-list-copy (copy-object voxel-list)))
    ;;(dolist (voxel voxel-list)
    ;;  (setf (get voxel :gl-displaylist-id) nil))
    (dolist (voxel voxel-list-copy)
      (send voxel :locate pos)
      (setf (get voxel :gl-displaylist-id) nil)) ;; reset :gl-displaylist-id
    voxel-list-copy))


;; move voxel list by specifying the center position of voxel list
(defun set-voxel-center (voxel-list pos) ;; ex. pos #f(100 100 100)
  (let ((voxel-center-now (midpoint 0.5 (get-voxel-origin voxel-list) (get-voxel-last voxel-list))))
    (move-voxel-list voxel-list (v- pos voxel-center-now))))


;; which voxel is in the position ? ex. return : (1 2 3)
(defun pos-to-voxel-num (voxel-list position) ;; ex. position #f(100 100 100)
  (mapcar #'(lambda (origin size pos) (floor (/ (- pos (- origin (/ size 2))) size)))
	  (fv2list (get-voxel-origin voxel-list))
	  (fv2list (get-voxel-size voxel-list))
	  (fv2list position)))


;; (erase-voxel (make-voxel-list) #f(-100 -100 -100))
(defun erase-voxel (voxel-list pos) ;; ex. pos #f(100 100 100)
  (let* ((voxel-list-copy (copy-object voxel-list))
	 (voxel-number (how-many-voxel voxel-list)) ;;how many voxels in each row
	 (voxel-num-list (pos-to-voxel-num voxel-list-copy pos))
	 (voxel-order (+ (* (elt voxel-num-list 0) 1)
			 (* (elt voxel-num-list 1) (elt voxel-number 0))
			 (* (elt voxel-num-list 2) (* (elt voxel-number 0) (elt voxel-number 1))))))

    (if (and (v< (car (get-voxel-range voxel-list)) pos)
	     (v< pos (cadr (get-voxel-range voxel-list))))
	(progn
	  (dolist (voxel voxel-list)
	    (setf (get voxel :gl-displaylist-id) nil)) ;; reset :gl-displaylist-id
	  ;; voxel-num is out of index ??
	  (if (< voxel-order (apply #'* voxel-number))
	      (send (elt voxel-list-copy voxel-order) :set-color :red 0.1))
	  voxel-list-copy)
      voxel-list-copy)))


;; how large the voxel-list is? (start([mm] [mm] [mm]) end([mm] [mm] [mm]))
(defun get-voxel-range (voxel-list)
  (list
   (v- (get-voxel-origin voxel-list) (scale 0.5 (get-voxel-size voxel-list)))
   (v+ (get-voxel-last voxel-list) (scale 0.5 (get-voxel-size voxel-list)))))


;; get how-many-voxel in each row, ex. 5 voxel in each row => return : (5 5 5)
(defun how-many-voxel (voxel-list)
  (let ((last-voxel))
    (setq last-voxel (car (last voxel-list)))
    (list (+ 1 (elt (send last-voxel :name) 0))
	  (+ 1 (elt (send last-voxel :name) 1))
	  (+ 1 (elt (send last-voxel :name) 2)))))


;; get list of whole voxel size list ([mm] [mm] [mm])
(defun get-voxel-size (voxel-list)
  (list2fv (mapcar #'(lambda (x) (floor x)) (cdar (send (car voxel-list) :csg)))))


;; get center position of (0 0 0) voxel
(defun get-voxel-origin (voxel-list)
  (send (car voxel-list) :worldpos))

;; get center position of last voxel
(defun get-voxel-last (voxel-list)
  (send (car (last voxel-list)) :worldpos))

;; float-vector to list
(defun fv2list (fv)
  (let ((l nil))
    (dotimes (i (cdar (send fv :slots)))
      (setq l (append l (list (elt fv i)))))
    l))

;; list to float-vector
(defun list2fv (l)
  (let ((temp ""))
    (dolist (x l) (setq temp (format nil "~A ~A" temp x)))
    (eval (read-from-string (format nil "(float-vector~A)" temp)))))


;; DO rossetlocal & roscore
;; erase voxel where fetch arm pass
(defun demo ()
  (require "package://fetcheus/fetch-interface.l")
  (fetch-init)
  (format t "done fetch-init~%")

  (let ((voxel-list (make-voxel-list 10 10 10 50))
	(end-coords))

    (setq voxel-list (move-voxel-list voxel-list #f(800 -450 800)))
    (setq end-coords (make-coords :pos #f(800 0 1300)))
    (send *fetch* :inverse-kinematics end-coords)
    (objects (cons *fetch* voxel-list))

    (dotimes (i 80)
      (send end-coords :translate #f(0 0 -5))
      (send *fetch* :inverse-kinematics end-coords)
      (setq voxel-list (erase-voxel voxel-list (send (send *fetch* :rarm-end-coords) :worldpos)))
      (print (send (send *fetch* :rarm-end-coords) :worldpos))
      ;;(send *irtviewer* :draw-objects)
      (objects (cons *fetch* voxel-list))
      (x::window-main-one)
      (unix:usleep 10000)) ;; 0.01 sec

    (dotimes (i 100)
      (send end-coords :translate #f(0 -5 0))
      (send *fetch* :inverse-kinematics end-coords)
      (setq voxel-list (erase-voxel voxel-list (send (send *fetch* :rarm-end-coords) :worldpos)))
      (print (send (send *fetch* :rarm-end-coords) :worldpos))
      ;;(send *irtviewer* :draw-objects)
      (objects (cons *fetch* voxel-list))
      (x::window-main-one)
      (unix:usleep 10000)))) ;; 0.01 sec
