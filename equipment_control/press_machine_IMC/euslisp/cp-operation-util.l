#!/usr/bin/env roseus

(ros::roseus "command_cp")
(unix::usleep (* 100 1000))
(ros::rate 1)

;; advertise
;; human flag
(ros::advertise "coldpress/write/operation_start"    std_msgs::Bool 1)
(ros::advertise "coldpress/write/operation_reset"    std_msgs::Bool 1)
(ros::advertise "coldpress/write/carry_in_complete"  std_msgs::Bool 1)
(ros::advertise "coldpress/write/carry_out_complete" std_msgs::Bool 1)
;; machine flag
(ros::advertise "coldpress/write/start_waiting"      std_msgs::Bool 1)
(ros::advertise "coldpress/write/fullauto_running"   std_msgs::Bool 1)
(ros::advertise "coldpress/write/carry_in_possible"  std_msgs::Bool 1)
(ros::advertise "coldpress/write/carry_out_possible" std_msgs::Bool 1)
(ros::advertise "coldpress/write/fullauto_end"       std_msgs::Bool 1)
;; control flag
(ros::advertise "coldpress/write/reset_press_force"  std_msgs::Bool 1)
;; setting
(ros::advertise "coldpress/write/press_force/setting" std_msgs::Int16 1)
(ros::advertise "coldpress/write/press_time/setting"  std_msgs::Int32 1)
;; complete
(ros::advertise "/complete_coldpress"  std_msgs::Bool 1)

;;subscribe
(setq *carry-out-possible-cp* nil)
(setq *press-force-setting-cp* nil)
(setq *press-time-setting-cp*  nil)
(setq *next-setting-cp*  nil)

(defun cb-cp (msg)
  (setq *carry-out-possible-cp* (send msg :data))
  )
(defun cb-press-force-setting-cp (msg)
  (let (tmp-data)
    (setq tmp-data (send msg :data))
    (setq *press-force-setting-cp* (/ tmp-data 100.0))  ;; convert to [kN]
  ))
(defun cb-press-time-setting-cp (msg)
  (setq *press-time-setting-cp* (send msg :data)))
(defun cb-next-setting-cp (msg)
  (setq *next-setting-cp* (send msg :data)))

(defun cb-coldpress (msg)
  (if (send msg :data)
      (start-operation-cp :run nil)))

(ros::subscribe "coldpress/read/carry_out_possible" std_msgs::Bool #'cb-cp)
(ros::subscribe "coldpress/read/press_force/setting" std_msgs::Int16 #'cb-press-force-setting-cp)
(ros::subscribe "coldpress/read/press_time/setting"  std_msgs::Int32 #'cb-press-time-setting-cp)
(ros::subscribe "next_setting_cp"  std_msgs::Float32MultiArray #'cb-next-setting-cp)
(ros::subscribe "/start_coldpress" std_msgs::Bool #'cb-coldpress)

;; variables
;; human flag
(setq msg-cp-operation-start-on     (instance std_msgs::Bool :init))
(setq msg-cp-operation-start-off    (instance std_msgs::Bool :init))
(setq msg-cp-operation-reset-on     (instance std_msgs::Bool :init))
(setq msg-cp-operation-reset-off    (instance std_msgs::Bool :init))
(setq msg-cp-carry-in-complete-on   (instance std_msgs::Bool :init))
(setq msg-cp-carry-in-complete-off  (instance std_msgs::Bool :init))
(setq msg-cp-carry-out-complete-on  (instance std_msgs::Bool :init))
(setq msg-cp-carry-out-complete-off (instance std_msgs::Bool :init))

(send msg-cp-operation-start-on    :data t)
(send msg-cp-operation-reset-on    :data t)
(send msg-cp-carry-in-complete-on  :data t)
(send msg-cp-carry-out-complete-on :data t)

(send msg-cp-operation-start-off    :data nil)
(send msg-cp-operation-reset-off    :data nil)
(send msg-cp-carry-in-complete-off  :data nil)
(send msg-cp-carry-out-complete-off :data nil)

;; machine flag
(setq msg-cp-start-waiting-off      (instance std_msgs::Bool :init))
(setq msg-cp-fullauto-running-off   (instance std_msgs::Bool :init))
(setq msg-cp-carry-in-possible-off  (instance std_msgs::Bool :init))
(setq msg-cp-carry-out-possible-off (instance std_msgs::Bool :init))
(setq msg-cp-fullauto-end-off       (instance std_msgs::Bool :init))

(send msg-cp-start-waiting-off      :data nil)
(send msg-cp-fullauto-running-off   :data nil)
(send msg-cp-carry-in-possible-off  :data nil)
(send msg-cp-carry-out-possible-off :data nil)
(send msg-cp-fullauto-end-off       :data nil)

;; control flag
(setq msg-cp-reset-press-force-on (instance std_msgs::Bool :init))
(send msg-cp-reset-press-force-on :data t)


;; functions
(defun reset-operation-cp ()
  ;; oneshot reset
  (warn "reset-operation~%")
  (ros::publish "coldpress/write/operation_reset" msg-cp-operation-reset-on)
  (unix::usleep (* 500 1000))

  (ros::publish "coldpress/write/operation_reset" msg-cp-operation-reset-off)
  (unix::usleep (* 500 1000))
  )

(defun reset-machine-flag-cp ()
  (warn "reset-machine-flag~%")
  (ros::publish "coldpress/write/start_waiting"      msg-cp-start-waiting-off)
  (ros::publish "coldpress/write/fullauto_running"   msg-cp-fullauto-running-off)
  (ros::publish "coldpress/write/carry_in_possible"  msg-cp-carry-in-possible-off)
  (ros::publish "coldpress/write/carry_out_possible" msg-cp-carry-out-possible-off)
  (ros::publish "coldpress/write/fullauto_end"       msg-cp-fullauto-end-off)
  )

(defun reset-human-flag-cp ()
  (warn "reset-human-flag~%")
  (ros::publish "coldpress/write/operation_start"    msg-cp-operation-start-off)
  (unix::usleep (* 100 1000))
  (ros::publish "coldpress/write/carry_in_complete"  msg-cp-carry-in-complete-off)
  (unix::usleep (* 100 1000))
  (ros::publish "coldpress/write/carry_out_complete" msg-cp-carry-out-complete-off)
  (unix::usleep (* 100 1000))
  )

(defun reset-press-force-cp ()
  (warn "reset-press-force~%")
  (ros::publish "coldpress/write/reset_press_force" msg-cp-reset-press-force-on)
  (unix::usleep (* 500 1000)))

(defun reset-all-cp ()
  (reset-operation-cp)
  (reset-human-flag-cp)
  (reset-machine-flag-cp)
  (reset-press-force-cp)
  )

(defun enable-operation-start-cp ()
  (ros::publish "coldpress/write/operation_start" msg-cp-operation-start-on)
  (warn "operation_start~%")
  (unix::usleep (* 500 1000))
  )

(defun enable-carry-in-complete-cp ()
  (ros::publish "coldpress/write/carry_in_complete" msg-cp-carry-in-complete-on)
  (warn "carry_in_complete~%")
  )

(defun enable-carry-out-complete-cp ()
  (ros::publish "coldpress/write/carry_out_complete" msg-cp-carry-out-complete-on)
  (warn "carry_out_complete~%")
  ;;
  (reset-human-flag-cp) ;; nowait reset within 500ms is neccessary
  (reset-operation-cp)
  )

;; setting
(defun change-setting-cp (press-force-kN press-time)
  (let (press-force
        (msg-press-force (instance std_msgs::Int16 :init))
        (msg-press-time  (instance std_msgs::Int32 :init)))
    ;; set
    (setq press-force (* press-force-kN 100))
    (send msg-press-force :data (round press-force))  ;; convert to integer
    (send msg-press-time  :data (round press-time))
    (ros::publish "coldpress/write/press_force/setting" msg-press-force)
    (ros::publish "coldpress/write/press_time/setting"  msg-press-time)
    (unix::usleep (* 100 1000))
    ;; check
    (ros::spin-once)
    (ros::sleep)
    (format t "change CP setting to:~%")
    (format t " press-force[kN]  : ~a~%" *press-force-setting-cp*)
    (format t " press-time [s]   : ~a~%" *press-time-setting-cp*)
    ))


;;;; machine operation ;;;;
(defun start-operation-cp (&key (run nil) (setting-press '(0.3 5)) (by-topic nil))
  (warn "(start-operation-cp)~%")
  (when run
    ;; setting
    (when by-topic
      (ros::spin-once)
      (ros::sleep)
      (setq setting-press (list
                           (elt *next-setting-cp* 1)
                           (elt *next-setting-cp* 2)))
      (format t "setting-press: ~a~%" setting-press))
    (when setting-press
      (unless (listp setting-press)
        (return-from start-operation-cp (warn "ERROR: setting is not a list~%"))))
    ;; main process of CP
    ;; setting
    (change-setting-cp (elt setting-press 0)
                       (elt setting-press 1))
    ;; reset
    (reset-all-cp)
    ;; flag on
    (enable-operation-start-cp)
    (enable-carry-in-complete-cp)
    ;;
    ;; wait until finish press
    (setq *carry-out-possible-cp* nil)  ;; this is necessary for only CP..
    (warn "wait until carry_out_possible turns true~%")
    (while (null *carry-out-possible-cp*)  ;; exit after carry_out_possible turns true
      (ros::spin-once)
      (ros::sleep)
      )
    (enable-carry-out-complete-cp)  ;; this flag enable is necessary to finish the operation.
    ) ;; run
  (warn "finish coldpress operation~%")
  )
