;; 1. Grayscale Conversion
;; The sources specify the exact weights for RGB to grayscale conversion.
(defun rgb-to-grayscale (r g b)
  "Converts RGB to Grayscale using the standard luminosity formula."
  (+ (* 0.299 r) (* 0.587 g) (* 0.114 b))) ; [5]

;; 2. Gaussian Blur Convolution
;; This removes high-frequency noise and minor variations before edge detection.
(defvar *gaussian-kernel* 
  '((1 2 1)
    (2 4 2)
    (1 2 1))) ; [6]

(defun apply-gaussian-blur (image width height)
  "Applies a 3x3 Gaussian blur kernel to a 2D grayscale image array."
  (let ((result (make-array (list height width) :initial-element 0)))
    (dotimes (y (- height 2))
      (dotimes (x (- width 2))
        (let ((yy (+ y 1))
              (xx (+ x 1))
              (sum 0))
          ;; Hardcoded 3x3 convolution based on the C-example provided in lecture [2]
          (setf sum (+ (* (aref image (- yy 1) (- xx 1)) 1)
                       (* (aref image (- yy 1) xx)       2)
                       (* (aref image (- yy 1) (+ xx 1)) 1)
                       (* (aref image yy (- xx 1))       2)
                       (* (aref image yy xx)             4)
                       (* (aref image yy (+ xx 1))       2)
                       (* (aref image (+ yy 1) (- xx 1)) 1)
                       (* (aref image (+ yy 1) xx)       2)
                       (* (aref image (+ yy 1) (+ xx 1)) 1)))
          ;; Divide by 16 as specified by the 3x3 kernel formula [6]
          (setf (aref result yy xx) (/ sum 16)))))
    result))

;; 3. Sobel Edge Detection & Thresholding
;; Sobel filters extract spatial gradients (edges) vertically and horizontally.
(defun apply-sobel-and-threshold (blurred-image width height threshold)
  "Applies Sobel operator for edge detection and thresholds the result."
  (let ((result (make-array (list height width) :initial-element 0)))
    (dotimes (y (- height 2))
      (dotimes (x (- width 2))
        (let ((yy (+ y 1))
              (xx (+ x 1))
              (gx 0)
              (gy 0)
              (g 0))
          
          ;; Apply Vertical Edge Kernel (Wx) [5]
          ;; [ 1  0 -1]
          ;; [ 2  0 -2]
          ;; [ 1  0 -1]
          (setf gx (+ (* (aref blurred-image (- yy 1) (- xx 1)) 1)
                      (* (aref blurred-image (- yy 1) (+ xx 1)) -1)
                      (* (aref blurred-image yy (- xx 1)) 2)
                      (* (aref blurred-image yy (+ xx 1)) -2)
                      (* (aref blurred-image (+ yy 1) (- xx 1)) 1)
                      (* (aref blurred-image (+ yy 1) (+ xx 1)) -1)))

          ;; Apply Horizontal Edge Kernel (Wy) [5]
          ;; [ 1  2  1]
          ;; [ 0  0  0]
          ;; [-1 -2 -1]
          (setf gy (+ (* (aref blurred-image (- yy 1) (- xx 1)) 1)
                      (* (aref blurred-image (- yy 1) xx) 2)
                      (* (aref blurred-image (- yy 1) (+ xx 1)) 1)
                      (* (aref blurred-image (+ yy 1) (- xx 1)) -1)
                      (* (aref blurred-image (+ yy 1) xx) -2)
                      (* (aref blurred-image (+ yy 1) (+ xx 1)) -1)))

          ;; Calculate final gradient magnitude: G = sqrt(Gx^2 + Gy^2) [5]
          (setf g (sqrt (+ (* gx gx) (* gy gy))))

          ;; 4. Thresholding 
          ;; If p > threshold, 255 (white edge), else 0 (black background) [1]
          (if (> g threshold)
              (setf (aref result yy xx) 255)
              (setf (aref result yy xx) 0)))))
    result))
