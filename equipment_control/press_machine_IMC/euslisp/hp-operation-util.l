#!/usr/bin/env roseus

(ros::roseus "command_hp")
(unix::usleep (* 100 1000))
(ros::rate 1)

;; advertise
;; human flag
(ros::advertise "heatpress/write/operation_start"    std_msgs::Bool 1)
(ros::advertise "heatpress/write/operation_reset"    std_msgs::Bool 1)
(ros::advertise "heatpress/write/carry_in_complete"  std_msgs::Bool 1)
(ros::advertise "heatpress/write/carry_out_complete" std_msgs::Bool 1)
;; machine flag
(ros::advertise "heatpress/write/start_waiting"      std_msgs::Bool 1)
(ros::advertise "heatpress/write/fullauto_running"   std_msgs::Bool 1)
(ros::advertise "heatpress/write/carry_in_possible"  std_msgs::Bool 1)
(ros::advertise "heatpress/write/carry_out_possible" std_msgs::Bool 1)
(ros::advertise "heatpress/write/fullauto_end"       std_msgs::Bool 1)
;; control flag
(ros::advertise "heatpress/write/reset_press_force"  std_msgs::Bool 1)
;; setting
(ros::advertise "heatpress/write/seg0/temp_top/setting"    std_msgs::Int16 1)
(ros::advertise "heatpress/write/seg0/temp_bottom/setting" std_msgs::Int16 1)
(ros::advertise "heatpress/write/seg0/press_force/setting" std_msgs::Int16 1)
(ros::advertise "heatpress/write/seg0/press_time/setting"  std_msgs::Int32 1)
;; complete
(ros::advertise "/complete_heatpress"  std_msgs::Bool 1)

;; subscribe
(setq *carry-out-possible-hp* nil)
(setq *temp-top-setting-hp*    nil)
(setq *temp-bottom-setting-hp* nil)
(setq *press-force-setting-hp* nil)
(setq *press-time-setting-hp*  nil)
(setq *next-setting-hp-pre*  nil)
(setq *next-setting-hp-main*  nil)

(defun cb-hp (msg)
  (setq *carry-out-possible-hp* (send msg :data))
  )
(defun cb-temp-top-setting-hp (msg)
  (setq *temp-top-setting-hp* (send msg :data)))
(defun cb-temp-bottom-setting-hp (msg)
  (setq *temp-bottom-setting-hp* (send msg :data)))
(defun cb-press-force-setting-hp (msg)
  (let (tmp-data)
    (setq tmp-data (send msg :data))
    (setq *press-force-setting-hp* (/ tmp-data 100.0))  ;; convert to [kN]
  ))
(defun cb-press-time-setting-hp (msg)
  (setq *press-time-setting-hp* (send msg :data)))
(defun cb-next-setting-hp-pre (msg)
  (setq *next-setting-hp-pre* (send msg :data)))
(defun cb-next-setting-hp-main (msg)
  (setq *next-setting-hp-main* (send msg :data)))

(defun cb-heatpress (msg)
  (if (send msg :data)
      (start-operation-with-pre-heat-hp :run nil)))

(ros::subscribe "heatpress/read/carry_out_possible" std_msgs::Bool #'cb-hp)
(ros::subscribe "heatpress/read/temp_top/setting"    std_msgs::Int16 #'cb-temp-top-setting-hp)
(ros::subscribe "heatpress/read/temp_bottom/setting" std_msgs::Int16 #'cb-temp-bottom-setting-hp)
(ros::subscribe "heatpress/read/press_force/setting" std_msgs::Int16 #'cb-press-force-setting-hp)
(ros::subscribe "heatpress/read/press_time/setting"  std_msgs::Int32 #'cb-press-time-setting-hp)
(ros::subscribe "next_setting_hp_pre"  std_msgs::Float32MultiArray #'cb-next-setting-hp-pre)
(ros::subscribe "next_setting_hp_main" std_msgs::Float32MultiArray #'cb-next-setting-hp-main)
(ros::subscribe "/start_heatpress"     std_msgs::Bool #'cb-heatpress)

;; variables
;; human flag
(setq msg-hp-operation-start-on     (instance std_msgs::Bool :init))
(setq msg-hp-operation-start-off    (instance std_msgs::Bool :init))
(setq msg-hp-operation-reset-on     (instance std_msgs::Bool :init))
(setq msg-hp-operation-reset-off    (instance std_msgs::Bool :init))
(setq msg-hp-carry-in-complete-on   (instance std_msgs::Bool :init))
(setq msg-hp-carry-in-complete-off  (instance std_msgs::Bool :init))
(setq msg-hp-carry-out-complete-on  (instance std_msgs::Bool :init))
(setq msg-hp-carry-out-complete-off (instance std_msgs::Bool :init))

(send msg-hp-operation-start-on    :data t)
(send msg-hp-operation-reset-on    :data t)
(send msg-hp-carry-in-complete-on  :data t)
(send msg-hp-carry-out-complete-on :data t)

(send msg-hp-operation-start-off    :data nil)
(send msg-hp-operation-reset-off    :data nil)
(send msg-hp-carry-in-complete-off  :data nil)
(send msg-hp-carry-out-complete-off :data nil)

;; machine flag
(setq msg-hp-start-waiting-off      (instance std_msgs::Bool :init))
(setq msg-hp-fullauto-running-off   (instance std_msgs::Bool :init))
(setq msg-hp-carry-in-possible-off  (instance std_msgs::Bool :init))
(setq msg-hp-carry-out-possible-off (instance std_msgs::Bool :init))
(setq msg-hp-fullauto-end-off       (instance std_msgs::Bool :init))

(send msg-hp-start-waiting-off      :data nil)
(send msg-hp-fullauto-running-off   :data nil)
(send msg-hp-carry-in-possible-off  :data nil)
(send msg-hp-carry-out-possible-off :data nil)
(send msg-hp-fullauto-end-off       :data nil)

;; control flag
(setq msg-hp-reset-press-force-on (instance std_msgs::Bool :init))
(send msg-hp-reset-press-force-on :data t)


;; functions
(defun reset-operation-hp ()
  ;; oneshot reset
  (warn "reset-operation~%")
  (ros::publish "heatpress/write/operation_reset" msg-hp-operation-reset-on)
  (unix::usleep (* 500 1000))

  (ros::publish "heatpress/write/operation_reset" msg-hp-operation-reset-off)
  (unix::usleep (* 500 1000))
  )

(defun reset-machine-flag-hp ()
  (warn "reset-machine-flag~%")
  (ros::publish "heatpress/write/start_waiting"      msg-hp-start-waiting-off)
  (ros::publish "heatpress/write/fullauto_running"   msg-hp-fullauto-running-off)
  (ros::publish "heatpress/write/carry_in_possible"  msg-hp-carry-in-possible-off)
  (ros::publish "heatpress/write/carry_out_possible" msg-hp-carry-out-possible-off)
  (ros::publish "heatpress/write/fullauto_end"       msg-hp-fullauto-end-off)
  )

(defun reset-human-flag-hp ()
  (warn "reset-human-flag~%")
  (ros::publish "heatpress/write/operation_start"    msg-hp-operation-start-off)
  (unix::usleep (* 100 1000))
  (ros::publish "heatpress/write/carry_in_complete"  msg-hp-carry-in-complete-off)
  (unix::usleep (* 100 1000))
  (ros::publish "heatpress/write/carry_out_complete" msg-hp-carry-out-complete-off)
  (unix::usleep (* 100 1000))
  )

(defun reset-press-force-hp ()
  (warn "reset-press-force~%")
  (ros::publish "heatpress/write/reset_press_force" msg-hp-reset-press-force-on)
  (unix::usleep (* 500 1000)))

(defun reset-all-hp ()
  (reset-operation-hp)
  (reset-human-flag-hp)
  (reset-machine-flag-hp)
  (reset-press-force-hp)
  )

(defun enable-operation-start-hp ()
  (ros::publish "heatpress/write/operation_start" msg-hp-operation-start-on)
  (warn "operation_start~%")
  (unix::usleep (* 500 1000))
  )

(defun enable-carry-in-complete-hp ()
  (ros::publish "heatpress/write/carry_in_complete" msg-hp-carry-in-complete-on)
  (warn "carry_in_complete~%")
  )

(defun enable-carry-out-complete-hp ()
  (ros::publish "heatpress/write/carry_out_complete" msg-hp-carry-out-complete-on)
  (warn "carry_out_complete~%")
  ;;
  (reset-human-flag-hp) ;; nowait reset within 500ms is neccessary
  (reset-operation-hp)
  )

;; setting
(defun change-setting-hp (temp-top temp-bottom press-force-kN press-time)
  (let (press-force
        (msg-temp-top    (instance std_msgs::Int16 :init))
        (msg-temp-bottom (instance std_msgs::Int16 :init))
        (msg-press-force (instance std_msgs::Int16 :init))
        (msg-press-time  (instance std_msgs::Int32 :init)))
    ;; set
    (setq press-force (* press-force-kN 100))
    (send msg-temp-top    :data (round temp-top))  ;; convert to integer
    (send msg-temp-bottom :data (round temp-bottom))
    (send msg-press-force :data (round press-force))
    (send msg-press-time  :data (round press-time))
    (ros::publish "heatpress/write/seg0/temp_top/setting"    msg-temp-top)
    (ros::publish "heatpress/write/seg0/temp_bottom/setting" msg-temp-bottom)
    (ros::publish "heatpress/write/seg0/press_force/setting" msg-press-force)
    (ros::publish "heatpress/write/seg0/press_time/setting"  msg-press-time)
    (unix::usleep (* 100 1000))
    ;; check
    (ros::spin-once)
    (ros::sleep)
    (format t "change HP setting to:~%")
    (format t " temp-top   [degC]: ~a~%" *temp-top-setting-hp*)
    (format t " temp-bottom[degC]: ~a~%" *temp-bottom-setting-hp*)
    (format t " press-force[kN]  : ~a~%" *press-force-setting-hp*)
    (format t " press-time [s]   : ~a~%" *press-time-setting-hp*)
    ))

;;;; machine operation ;;;;
;; (warn "(start-operation-hp :run t :setting-press '(30 30 0.3 10) :by-topic nil)")
(defun start-operation-hp (&key (run nil) (setting-press '(20 20 0.3 5)) (by-topic nil))
  (warn "(start-operation-hp)~%")
  ;; setting
  (when by-topic
    (ros::spin-once)
    (ros::sleep)
    (setq setting-press (list
                         (elt *next-setting-hp-main* 1)
                         (elt *next-setting-hp-main* 2)
                         (elt *next-setting-hp-main* 3)
                         (elt *next-setting-hp-main* 4)))
    (format t "setting-press: ~a~%" setting-press))
  (when setting-press
    (unless (listp setting-press)
      (return-from start-operation-hp (warn "ERROR: setting is not a list~%")))
    (change-setting-hp (elt setting-press 0)
                       (elt setting-press 1)
                       (elt setting-press 2)
                       (elt setting-press 3)))
  (when run
    ;; reset
    (reset-all-hp)
    ;; flag on
    (enable-operation-start-hp)
    (enable-carry-in-complete-hp)
    ;; wait until finish press
    (warn "wait until temp rise (carry_out_possible turns true)~%")
    (while (null *carry-out-possible-hp*)  ;; exit after carry_out_possible turns true
      (ros::spin-once)
      (ros::sleep)
      )
    (enable-carry-out-complete-hp)  ;; this flag enable is necessary to finish the operation.
    )
  (warn "finish heatpress operation~%")
  )

(defun start-operation-with-pre-heat-hp (&key (run nil) (setting-pre-heat '(180 180 0.05 5)) (setting-press '(180 180 0.3 5)) (by-topic nil))
  (warn "(start-operation-with-pre-heat-hp)~%")
  (when run
    ;; setting
    (when by-topic
      (ros::spin-once)
      (ros::sleep)
      (setq setting-pre-heat (list
                              (elt *next-setting-hp-pre* 1)
                              (elt *next-setting-hp-pre* 2)
                              (elt *next-setting-hp-pre* 3)
                              (elt *next-setting-hp-pre* 4)))
      (setq setting-press (list
                           (elt *next-setting-hp-main* 1)
                           (elt *next-setting-hp-main* 2)
                           (elt *next-setting-hp-main* 3)
                           (elt *next-setting-hp-main* 4)))
      (format t "setting-pre-heat: ~a~%" setting-pre-heat)
      (format t "setting-main   : ~a~%" setting-press))
    (when setting-pre-heat
      (unless (listp setting-pre-heat)
        (return-from start-operation-with-pre-heat-hp (warn "ERROR: setting-pre-heat is not a list~%"))))
    (when setting-press
      (unless (listp setting-press)
        (return-from start-operation-with-pre-heat-hp (warn "ERROR: setting-press is not a list~%"))))
    ;; pre-heat process
    ;; setting
    (change-setting-hp (elt setting-pre-heat 0)
                       (elt setting-pre-heat 1)
                       (elt setting-pre-heat 2)
                       (elt setting-pre-heat 3))
    ;; reset
    (reset-all-hp)
    ;; flag on
    (enable-operation-start-hp)
    (enable-carry-in-complete-hp)
    ;; wait until finish press
    (warn "wait until temp rise (carry_out_possible turns true)~%")
    (while (null *carry-out-possible-hp*)  ;; exit after carry_out_possible turns true
      (ros::spin-once)
      (ros::sleep)
      )
    (enable-carry-out-complete-hp)  ;; this flag enable is necessary to finish the operation.
    ;;
    ;; main-press process
    ;; setting
    (change-setting-hp (elt setting-press 0)
                       (elt setting-press 1)
                       (elt setting-press 2)
                       (elt setting-press 3))
    ;; reset
    (reset-all-hp)
    ;; flag on
    (enable-operation-start-hp)
    (enable-carry-in-complete-hp)
    ;; wait until finish press
    (warn "wait until temp rise (carry_out_possible turns true)~%")
    (while (null *carry-out-possible-hp*)
      (ros::spin-once)
      (ros::sleep)
      )
    (enable-carry-out-complete-hp)  ;; this flag enable is necessary to finish the operation.
    ) ;; run
  ;; publish complete flag
  (setq msg (instance std_msgs::Bool :init))
  (send msg :data t)
  (ros::publish "/complete_heatpress" msg)
  (warn "finish heatpress operation~%")
  )
