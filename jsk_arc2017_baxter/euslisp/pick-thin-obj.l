#!/usr/bin/env roseus

(ros::roseus "pick-thin-obj")
(require "package://jsk_arc2017_baxter/euslisp/lib/baxter-interface.l")

(defun init()
  (jsk_arc2017_baxter::baxter-init)
)

(defun demo()
  (if (null (boundp '*baxter*)) (init))
  (send *baxter* :reset-pose)
  (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (let*
      ((c (send (make-cube 300 300 50) :translate #f(800 0 0)))
       (co-r (send c :copy-worldcoords))
       (co-l (send
              (send (send c :copy-worldcoords) :rotate -pi/2 :x)
              :rotate pi/2 :z)))
    (send c :set-color #f(0 0 255) 0.3)
    (objects (list *baxter* c))
    (send *irtviewer* :change-background #f(0.5 0.5 0.5))
    (send *irtviewer* :draw-objects)

    (send *baxter* :rarm :inverse-kinematics
          (send co-r :translate #f(0 0 25) :world)
          :use-gripper t
          :rotation-axis :z)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (send *baxter* :larm :inverse-kinematics
          (send co-l :translate #f(0 100 25) :world)
          :use-gripper t
          :rotation-axis t)
    (send *baxter* :hand :larm :angle-vector #f(90 90))
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)

    (send (send *baxter* :larm :end-coords) :assoc c)
    (send *baxter* :rarm :move-end-pos #f(0 0 100) :world)
    (send *baxter* :larm :move-end-pos #f(0 0 100) :world)
    (send *ri* :angle-vector-raw (send *baxter* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *irtviewer* :draw-objects)
    ))
