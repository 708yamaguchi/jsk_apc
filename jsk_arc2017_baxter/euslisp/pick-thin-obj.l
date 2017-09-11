#!/usr/bin/env roseus

(ros::roseus "pick-thin-obj")
(require "package://jsk_arc2017_baxter/euslisp/lib/baxter-interface.l")

(defun start()
  (jsk_arc2017_baxter::baxter-init)
  (setq c (make-cube 300 300 50))
  (send c :translate #f(800 0 0))
  (send c :set-color #f(0 0 255) 0.3)
  (objects (list *baxter* c))
  (send *irtviewer* :change-background #f(0.5 0.5 0.5))
)

(defun test()
  (if (null (boundp '*baxter*)) (start))
  (send *baxter* :reset-pose)
  (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (let ((co-r (send (send c :copy-worldcoords) :worldcoords))
        (co-l (send
               (send
                (send (send c :copy-worldcoords) :rotate -pi/2 :x)
                :rotate pi/2 :z)
               :locate #f(0 0 100))))
    ;; (send c :locate #f(0 0 -50))
    (send *baxter* :rarm :inverse-kinematics
          co-r
          :use-gripper t
          :rotation-axis :z)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *baxter* :larm :inverse-kinematics
          co-l
          :use-gripper t
          :rotation-axis t)
    (send *baxter* :hand :larm :angle-vector #f(90 90))
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    ))
