(ql:quickload :sketch)

(defpackage :nematode (:use :cl :sketch))
(in-package :nematode)


;here, we start nematode simulation
(defvar x 400)
(defvar y 300)
;angle in radians
(defvar v 0)

(defun turn-left ()
	(setf v (+ v (/ pi 6)))
)
(defun turn-right ()
	(setf v (- v (/ pi 6)))
)

;hunger state
(defvar hunger 0)

;center of food coordinates
(defvar fx1 600)
(defvar fy1 400)

(defun distance-to-food ()
	(let
		(
			(distance 
				(sqrt
					(+
						(* (- x fx1) (- x fx1))
						(* (- y fy1) (- y fy1))
					)
				)
			)
		)
		distance
	)
)


(defun left-distance-to-food ()
	(let*
		(
			(v-left (+ v (/ pi 3)))
			(distance 
				(sqrt
					(+
						(* (- (+ x (cos v-left)) fx1) (- (+ x (cos v-left)) fx1))
						(* (- (+ y (sin v-left)) fy1) (- (+ y (sin v-left)) fy1))
					)
				)
			)
		)
		distance
	)
)

(defun right-distance-to-food ()
	(let*
		(
			(v-right (- v (/ pi 3)))
			(distance 
				(sqrt
					(+
						(* (- (+ x (cos v-right)) fx1) (- (+ x (cos v-right)) fx1))
						(* (- (+ y (sin v-right)) fy1) (- (+ y (sin v-right)) fy1))
					)
				)
			)
		)
		distance
	)
)

(defun update-nematode-pos ()
	(setf x (+ x (cos v)))
	(when (> x 800)
		(setf x 800)
		;(turn)
	)
	(when (< x 0)
		(setf x 0)
		;(turn)
	)
	(setf y (+ y (sin v)))
	(when (> y 600)
		(setf y 600)
		;(turn)
	)
	(when (< y 0)
		(setf y 0)
		;(turn)
	)
)

;nematode's perception of how far away it is from food
(defvar food-perception (distance-to-food))
;nematode's perception of how its hunger is changing
(defvar hunger-perception hunger)



(defparameter +threshold-min+ 10)
(defparameter +threshold-max+ 30)

(defun make-neuron ()
	(let
		(
			(spikes 0)
			(threshold +threshold-min+)
			(fire nil)
		)
		(list
			;spike the neuron
			(lambda ()
				(incf spikes)
				(when (> spikes threshold)
					(setf spikes 0)
					(dolist (fn fire)
						(funcall fn)
					)
				)
			)
			;excite
			(lambda ()
				(unless (equal threshold +threshold-min+)
					(decf threshold)
				)
			)
			;inhibit
			(lambda ()
				(unless (equal threshold +threshold-max+)
					(incf threshold)
				)
			)
			;connect to other neurons
			(lambda (firings)
				(setf fire firings)
			)
		)
	)
)

(defun spike (neuron)
	(funcall (nth 0 neuron))
)
(defun excite (neuron)
	(funcall (nth 1 neuron))
)
(defun inhibit (neuron)
	(funcall (nth 2 neuron))
)
(defun connect (neuron spike-lst)
	(funcall (nth 3 neuron) spike-lst)
)

;let's create the nematode
(let
	(
		;positive food smell to the right
		(n-right (make-neuron))
		;positive food smell to the left
		(n-left (make-neuron))
		;forward movement
		(n-forward (make-neuron))
	)
	;senses increase in food smell to the right, turns  and excites n-forward, inhibits turn left
	(connect n-right (list 'turn-right (nth 1 n-forward) (nth 2 n-left)))
	;senses increase in food smell to the left, turns  and excites n-forward
	(connect n-left (list 'turn-left (nth 1 n-forward) (nth 2 n-right)))
	;n3 moves forward: triggers movement function
	(connect n-forward (list 'update-nematode-pos))
	

	(defun nematode-act ()
		;update hunger to reflect where we are
		(if (< (distance-to-food) 100)
			(when (> hunger 0)
				(decf hunger)
			)
			(incf hunger)
		)
		; ;if we are getting closer, spike n1; else, spike n2
		(if (< (right-distance-to-food) (left-distance-to-food))
			(spike n-right)
			(spike n-left)
		)
		(spike n-forward)
		(setf food-perception (distance-to-food))
	)
)




(defvar food 
	(list 
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
		(list (random 100) (random 100))
	)
)






(defun draw-label ()
  (with-font (make-font 
  							:color +blue+
               	:size 12
               	:line-height 1
               	:align :left
               )
   	(text (format nil "Hunger: ~a" hunger) 25 5)
  )
)




(defun draw-nematode (x y angle)
	;velocity is an x,y vector of length 1
	(with-rotate (angle x y)
		(with-pen (make-pen :fill +black+ :winding-rule :odd)
	  	(ellipse x y 10 5)  
	  )
	)
)	




(defun draw-food ()
	(with-pen (make-pen :fill +green+ :winding-rule :odd)
		(dolist (morsel food)
			(circle (+ fx1 (nth 0 morsel) (- 50)) (+ fy1 (nth 1 morsel) (- 50)) 10)
		)
  )
)

(defvar px1 (random 800))
(defvar px2 (random 800))
(defvar px3 (random 800))
(defvar py1 (random 600))
(defvar py2 (random 600))
(defvar py3 (random 600))

(defun draw-poison ()
	(with-pen (make-pen :fill +red+ :winding-rule :odd)
  	(circle px1 py1 20)
  	(circle px2 py2 20)
  	(circle px3 py3 20)  
  )
)	





(defun rad->deg (x) (* x (/ 180 pi)))


(defsketch grid
  (
    (width 800)
    (height 600)
    (copy-pixels nil)
		(frame-ctr 0)
  )
  (background +white+)
  ; (with-pen (make-pen :fill +black+ :winding-rule :odd)
  ; 	(ellipse 200 200 100 50)  
  ; )

  (draw-food)
  (draw-poison)

  (draw-nematode x y (rad->deg v))

  (draw-label)

  (nematode-act)
)

(make-instance 'grid)