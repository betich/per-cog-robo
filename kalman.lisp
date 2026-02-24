(ql:quickload :sketch)
(ql:quickload "gsll")


(defpackage :kalman (:use :cl :sketch))
(in-package :kalman)

(defvar robot 	(list 0 300)) ; position, x y
(defvar robot-k (list 0 300)) ;kalman result
(defvar robot-p (list 0 300)) ; position, x y
(defvar robot-s (list 0 300)) ; position, x y
(defvar velocity 0) ;velocity
(defvar velocity-k 0) ;velocity 
(defvar velocity-p 0) ;velocity
(defvar velocity-s 0) ;velocity  
(defvar acceleration (- 0.5d0))
(defvar acceleration-p (- 0.5d0))   

(defvar process-noise nil)
(defvar sensor-noise nil)
(defvar kalman-on nil)

(defparameter *rng* (gsll:make-random-number-generator gsll:+mt19937+))
(defparameter *process-sigma* 0.5d0)
(defparameter *sensor-sigma* 15.4d0)

(defun draw-label ()
	(with-pen (make-pen :fill +blue+ :winding-rule :odd)
    (polygon 
      (- 10 10) (- 10 10) 
      (+ 10 10) 10 
      (- 10 10) (+ 10 10) 
      (- 10 2) 10
    )
  )
  (with-pen (make-pen :fill +red+ :winding-rule :odd)
    (polygon 
      (- 10 10) (- 30 10) 
      (+ 10 10) 30 
      (- 10 10) (+ 30 10) 
      (- 10 2) 30
    )
  )
  (with-pen (make-pen :fill +green+ :winding-rule :odd)
    (polygon 
      (- 10 10) (- 50 10) 
      (+ 10 10) 50 
      (- 10 10) (+ 50 10) 
      (- 10 2) 50
    )
  )
  (with-pen (make-pen :fill +magenta+ :winding-rule :odd)
    (polygon 
      (- 10 10) (- 70 10) 
      (+ 10 10) 70 
      (- 10 10) (+ 70 10) 
      (- 10 2) 70
    )
  )
  (with-font (make-font 
  							:color +blue+
               	:size 12
               	:line-height 1
               	:align :left
               )
   	(text (format nil "Real: ~a" (nth 1 robot)) 25 5)
  )
  (with-font (make-font 
  							:color +red+
               	:size 12
               	:line-height 1
               	:align :left
               )
   	(text (format nil "Predicted: ~a" (nth 1 robot-p)) 25 25)
  )
  (with-font (make-font 
  							:color +green+
               	:size 12
               	:line-height 1
               	:align :left
               )
   	(text (format nil "Sensed: ~a" (nth 1 robot-s)) 25 45)
  )
  (with-font (make-font 
  							:color +magenta+
               	:size 12
               	:line-height 1
               	:align :left
               )
   	(text (format nil "Kalman: ~a" (nth 1 robot-k)) 25 65)
  )
)

(defvar robot-line nil)
(defvar robot-line-s nil)
(defvar robot-line-p nil)
(defvar robot-line-k nil)

(defun draw-robot (x y)
  (with-pen (make-pen :fill +blue+ :winding-rule :odd)
    (polygon 
      (- x 10) (- y 10) 
      (+ x 10) y 
      (- x 10) (+ y 10) 
      (- x 2) y
    )
    ;(line (car pos) (cdr pos) (car new-pos) (cdr new-pos))
  )
  (with-pen (make-pen :stroke +blue+ :fill +blue+ :weight 1)
  	(unless (<= (list-length robot-line) 2)
    	(dotimes (i (- (list-length robot-line) 1))
    		(line (nth 0 (nth i robot-line)) (nth 1 (nth i robot-line))
    					(nth 0 (nth (+ i 1) robot-line)) (nth 1 (nth (+ i 1) robot-line))
    		)
    	)
    )
  )
)

(defun draw-kalman (x y)
	;(format t "Kalman: ~a,~a~%" x y)
  (with-pen (make-pen :fill +magenta+ :winding-rule :odd)
    (polygon 
      (- x 10) (- y 10) 
      (+ x 10) y 
      (- x 10) (+ y 10) 
      (- x 2) y
    )
  )
  (with-pen (make-pen :stroke +magenta+ :fill +blue+ :weight 3)
  	(unless (<= (list-length robot-line-k) 2)
    	(dotimes (i (- (list-length robot-line-k) 1))
    		(line (nth 0 (nth i robot-line-k)) (nth 1 (nth i robot-line-k))
    					(nth 0 (nth (+ i 1) robot-line-k)) (nth 1 (nth (+ i 1) robot-line-k))
    		)
    	)
    )
  )
)

(defun draw-prediction (x y)
  (with-pen (make-pen :fill +red+ :winding-rule :odd)
    (polygon 
      (- x 10) (- y 10) 
      (+ x 10) y 
      (- x 10) (+ y 10) 
      (- x 2) y
    )
  )
)
(defun draw-sensor (x y)
  (with-pen (make-pen :fill +green+ :winding-rule :odd)
    (polygon 
      (- x 10) (- y 10) 
      (+ x 10) y 
      (- x 10) (+ y 10) 
      (- x 2) y
    )
  )
  (with-pen (make-pen :stroke +green+ :fill +blue+ :weight 1)
  	(unless (<= (list-length robot-line-s) 2)
    	(dotimes (i (- (list-length robot-line-s) 1))
    		(line (nth 0 (nth i robot-line-s)) (nth 1 (nth i robot-line-s))
    					(nth 0 (nth (+ i 1) robot-line-s)) (nth 1 (nth (+ i 1) robot-line-s))
    		)
    	)
    )
  )
)



(defun update-acceleration ()
	(if (> (nth 1 robot) 300)
		(setf acceleration -0.5d0)
		(setf acceleration 0.5d0)
	)
	(when process-noise
		(setf acceleration 
			(+ 
				acceleration
				(gsll:sample *rng* :gaussian :sigma *process-sigma*)
			)
		)
	)
)
(defun update-acceleration-p ()
	(if (> (nth 1 robot) 300)
		(setf acceleration-p -0.5d0)
		(setf acceleration-p 0.5d0)
	)
)


(defun update-velocity ()
	(setf velocity (+ velocity acceleration))
	(if (> velocity 10)
		(setf velocity 10)
	)
	(if (< velocity (- 10))
		(setf velocity (- 10))
	)
)
(defun update-velocity-p ()
	(if kalman-on
		(setf velocity-p (+ velocity-k acceleration-p))
		(setf velocity-p (+ velocity-s acceleration-p))
	)
	
	(if (> velocity-p 10)
		(setf velocity-p 10)
	)
	(if (< velocity-p (- 10))
		(setf velocity-p (- 10))
	)
)

(defun update-pos ()
	(if (equal (nth 0 robot) 800)
		(progn
			(setf (nth 0 robot) 0)
			(setf robot-line nil)
		)
		(incf (nth 0 robot))
	)
	(setf (nth 1 robot) (+ (nth 1 robot) velocity))
	(setf robot-line (append (list (copy-tree robot)) robot-line ))
)

(defun update-pos-p ()
	(if (equal (nth 0 robot-s) 800)
		(progn
			(setf (nth 0 robot-p) 0)
			(setf robot-line-p nil)
		)
		(incf (nth 0 robot-p))
	)
	(setf (nth 1 robot-p) (+ (nth 1 robot-s) velocity-p))
	(setf robot-line-p (append (list (copy-tree robot-p)) robot-line-p ))
)

(defun update-sensor ()
	(if (equal (nth 0 robot-s) 800)
		(setf robot-line-s nil)
	)
	(setf robot-s (copy-tree robot))
	(setf velocity-s velocity)
	(when sensor-noise
		(setf (nth 1 robot-s) 
			(+ 
				(nth 1 robot-s)
				(gsll:sample *rng* :gaussian :sigma *sensor-sigma*)
			)
		)
	)
	(setf robot-line-s (append (list (copy-tree robot-s)) robot-line-s ))
)



(defvar P-e (list (list 0 0) (list 0 0)))
(defvar P-p (list (list 0 0)(list 0 0)))
(defvar A (list (list 1 1)(list 0 1)))
(defvar K (list (list 1 0)(list 0 1)))
(defvar X-e (list (list 1) (list 1)))
(defvar X-p (list (list 1)(list 1)))
(defvar Q (list
							(list *process-sigma* (* *process-sigma* *process-sigma*))
							(list (* *process-sigma* *process-sigma*) *process-sigma*)
						)
)
(defvar R (list
							(list *sensor-sigma* 0)
							(list 0 0)
						)
)

(defun matrix-2x2-add (A B)
	(mapcar
		(lambda (rowA rowB)
			(mapcar
				(lambda (a b)
					(+ a b)
				)
				rowA
				rowB
			)
		)
		A
		B
	)
)
(defun matrix-2x2-sub (A B)
	(mapcar
		(lambda (rowA rowB)
			(mapcar
				(lambda (a b)
					(- a b)
				)
				rowA
				rowB
			)
		)
		A
		B
	)
)
;(matrix-2x2-mul (list (list 1 2) (list 3 4)) (list (list 4 5) (list 6 7)))
(defun matrix-2x2-mul (A B)
	(let
		(
			(C (list (list 0 0) (list 0 0)))
		)
		;first index is column
		(setf (nth 0 (nth 0 C))
			(+
				(* (nth 0 (nth 0 A)) (nth 0 (nth 0 B)))
				(* (nth 1 (nth 0 A)) (nth 0 (nth 1 B))) 
			)
		)
		(setf (nth 1 (nth 0 C))
			(+
				(* (nth 0 (nth 0 A)) (nth 1 (nth 0 B)))
				(* (nth 1 (nth 0 A)) (nth 1 (nth 1 B))) 
			)
		)
		(setf (nth 0 (nth 1 C))
			(+
				(* (nth 0 (nth 1 A)) (nth 0 (nth 0 B)))
				(* (nth 1 (nth 1 A)) (nth 0 (nth 1 B))) 
			)
		)
		(setf (nth 1 (nth 1 C))
			(+
				(* (nth 0 (nth 1 A)) (nth 1 (nth 0 B)))
				(* (nth 1 (nth 1 A)) (nth 1 (nth 1 B))) 
			)
		)
		C
	)
)

(defun matrix-2x1-mul (A B)
	(let
		(
			(C (list (list 0) (list 0)))
		)
		;first index is column
		(setf (nth 0 (nth 0 C))
			(+
				(* (nth 0 (nth 0 A)) (nth 0 (nth 0 B)))
				(* (nth 1 (nth 0 A)) (nth 0 (nth 1 B))) 
			)
		)
		(setf (nth 0 (nth 1 C))
			(+
				(* (nth 0 (nth 1 A)) (nth 0 (nth 0 B)))
				(* (nth 1 (nth 1 A)) (nth 0 (nth 1 B))) 
			)
		)		
		C
	)
)

(defun matrix-2x2-transpose (A)
	(list 
		(list (nth 0 (nth 0 A)) (nth 0 (nth 1 A)))
		(list (nth 1 (nth 0 A)) (nth 1 (nth 1 A)))
	)
)


(defun matrix-2x2-inverse (A)
	(let
		(
			(det 
				(-
					(* (nth 0 (nth 0 A)) (nth 1 (nth 1 A)))
					(* (nth 1 (nth 0 A)) (nth 0 (nth 1 A)))
				)
			)
		)
		(list 
			(list (/ (nth 1 (nth 1 A)) det) (- 0 (/ (nth 1 (nth 0 A)) det)))
			(list (- 0 (/ (nth 0 (nth 1 A)) det)) (/ (nth 0 (nth 0 A)) det))
		)
	)
)

(defun kalman ()
;xp_k = prediction over xe_{k-1}

	(setf X-p (matrix-2x1-mul A X-e))

	(setf (nth 0 (nth 1 X-p)) (+ (nth 0 (nth 1 X-p)) acceleration-p))

	(if (> (nth 0 (nth 1 X-p)) 10)
		(setf (nth 0 (nth 1 X-p)) 10)
	)
	(if (< (nth 0 (nth 1 X-p)) (- 10))
		(setf (nth 0 (nth 1 X-p)) (- 10))
	)

;Pp_k = APe_{k-1} A^T + Q

	(setf P-p 
		(matrix-2x2-add 
			(matrix-2x2-mul
				(matrix-2x2-mul
					A
					P-e
				)
				(matrix-2x2-transpose A)
			)
			Q
		)
	)

;K_k = Pp_{k} ( Pp_k  + R)^{-1}

	(setf K
		(matrix-2x2-mul
			P-p
			(matrix-2x2-inverse
				(matrix-2x2-add P-p R)
			)
		)
	)
	
;xe_k = xp_k + K_k (z_k - xp_k)

	(setf X-e
		(matrix-2x2-add
			X-p
			(matrix-2x1-mul
				K
				(matrix-2x2-sub
					(list (list (nth 1 robot-s)) (list velocity-s))
					X-p
				)
			)
		)
	)

	(if (> (nth 0 (nth 1 X-e)) 10)
		(setf (nth 0 (nth 1 X-e)) 10)
	)
	(if (< (nth 0 (nth 1 X-e)) (- 10))
		(setf (nth 0 (nth 1 X-e)) (- 10))
	)

;Pe_k = Pp_k - K_k Pp_k

	(setf P-e
		(matrix-2x2-sub
			P-p
			(matrix-2x2-mul
				K
				P-p
			)
		)
	)
	
	(setf robot-k (list (nth 0 robot) (nth 0 (nth 0 X-e))))
	(setf velocity-k (nth 0 (nth 1 X-e)))
	
	(if (equal (nth 0 robot) 800)
		(setf robot-line-k nil)
		(setf robot-line-k (append (list (copy-tree robot-k)) robot-line-k ))
	)
)



(defsketch grid
  (
    (width 800)
    (height 600)
    (copy-pixels t)
		(frame-ctr 0)
  )
  (background +white+)
  (draw-label)
  (draw-sensor (nth 0 robot-s) (nth 1 robot-s))
  (draw-prediction (nth 0 robot-p) (nth 1 robot-p))
 
 	(when kalman-on
 		(draw-kalman (nth 0 robot-k) (nth 1 robot-k))
 	)
  (draw-robot (nth 0 robot) (nth 1 robot))
   
  
  (update-pos-p)
  (update-velocity-p)
  (update-acceleration-p)

	;update real 
  	;potentially with process noise
  (update-pos)
	(update-velocity)
  (update-acceleration)
  
  ;update sensor based on real
  	;potentially, with sensor noise
  (update-sensor)

  (when kalman-on
 		(kalman)
 	)
)

(make-instance 'grid)



(defun process-noise-on ()
	(setf process-noise t)
)
(defun process-noise-off ()
	(setf process-noise nil)
)
(defun sensor-noise-on ()
	(setf sensor-noise t)
)
(defun sensor-noise-off ()
	(setf sensor-noise nil)
)

(defun kalman-on ()
	(setf robot-k (copy-tree robot))
	(setf (nth 0 (nth 0 X-e)) (nth 1 robot))
	(setf (nth 0 (nth 1 X-e)) velocity)
	(setf kalman-on t)
	(format t "Kalman now on~%")
)

(defun kalman-off ()
	(setf kalman-on nil)
	(setf robot-line-k nil)
)
