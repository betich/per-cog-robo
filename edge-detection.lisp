;; Edge Detection Implementation
;; Load image library (optional - only needed for loading image files)
;; To enable image loading: (ql:quickload :opticl)

;; 0. Image File Loading (requires opticl library)

(defun resize-image (image scale-factor)
  "Resizes an image by the given scale factor using nearest-neighbor sampling.
   scale-factor should be between 0 and 1 (e.g., 0.5 for half size)."
  (let* ((old-height (array-dimension image 0))
         (old-width (array-dimension image 1))
         (new-height (max 1 (floor (* old-height scale-factor))))
         (new-width (max 1 (floor (* old-width scale-factor))))
         (new-image (make-array (list new-height new-width) :initial-element 0)))
    
    (dotimes (y new-height)
      (dotimes (x new-width)
        (let ((src-y (min (floor (/ y scale-factor)) (1- old-height)))
              (src-x (min (floor (/ x scale-factor)) (1- old-width))))
          (setf (aref new-image y x) (aref image src-y src-x)))))
    
    (format t "✓ Resized from ~dx~d to ~dx~d pixels~%" 
            old-width old-height new-width new-height)
    new-image))

(defun calculate-safe-scale (width height &optional (max-pixels 1000000))
  "Calculate scale factor to keep total pixels under max-pixels.
   Returns 1.0 if image is already small enough."
  (let ((current-pixels (* width height)))
    (if (<= current-pixels max-pixels)
        1.0
        (sqrt (/ max-pixels current-pixels)))))

(defun load-image-file (filepath &key (max-size nil) (auto-scale t))
  "Loads an image file (JPG, PNG, etc.) and converts to grayscale array.
   
   IMPORTANT: First install opticl with: (ql:quickload :opticl)
   
   Options:
     :max-size - Maximum dimension (width or height), e.g., 800
     :auto-scale - Automatically scale down if image > 1M pixels (default: t)
   
   Usage: 
     (load-image-file \"photo.jpg\")                    ; Auto-scale if too large
     (load-image-file \"photo.jpg\" :max-size 800)       ; Limit to 800px max
     (load-image-file \"photo.jpg\" :auto-scale nil)     ; No auto-scaling
   
   Returns a 2D grayscale array suitable for edge detection."
  (handler-case
      (progn
        ;; Check if opticl is available
        (unless (find-package :opticl)
          (format t "~%ERROR: OPTICL package not found!~%")
          (format t "Please install it first with:~%")
          (format t "  (ql:quickload :opticl)~%~%")
          (error "OPTICL not installed"))
        
        ;; Load the image
        (let* ((color-image (funcall (intern "READ-IMAGE-FILE" :opticl) filepath))
               (height (array-dimension color-image 0))
               (width (array-dimension color-image 1))
               (grayscale (make-array (list height width) :initial-element 0)))
          
          ;; Convert to grayscale
          (dotimes (y height)
            (dotimes (x width)
              (let ((r (aref color-image y x 0))
                    (g (aref color-image y x 1))
                    (b (aref color-image y x 2)))
                (setf (aref grayscale y x) 
                      (round (rgb-to-grayscale r g b))))))
          
          (format t "✓ Loaded image: ~a (~dx~d pixels)~%" filepath width height)
          
          ;; Apply scaling if needed
          (let ((final-image grayscale))
            ;; Option 1: Scale by max dimension
            (when max-size
              (let ((max-dim (max width height)))
                (when (> max-dim max-size)
                  (let ((scale (/ max-size max-dim)))
                    (format t "  Scaling down to fit ~d pixel limit...~%" max-size)
                    (setf final-image (resize-image final-image scale))))))
            
            ;; Option 2: Auto-scale if too large
            (when (and auto-scale (not max-size))
              (let ((scale (calculate-safe-scale 
                           (array-dimension final-image 1)
                           (array-dimension final-image 0)
                           1000000))) ; 1M pixels default limit
                (when (< scale 1.0)
                  (format t "  Image too large! Auto-scaling to ~d%...~%" 
                          (round (* scale 100)))
                  (setf final-image (resize-image final-image scale)))))
            
            final-image)))
    (error (e)
      (format t "~%ERROR: Could not load image!~%")
      (format t "Reason: ~a~%~%" e)
      (format t "To fix:~%")
      (format t "1. Install opticl: (ql:quickload :opticl)~%")
      (format t "2. Make sure the file exists: ~a~%~%" filepath)
      nil)))

(defun process-image-file (filepath &key (threshold 50) (output-path nil) (max-size nil) (auto-scale t))
  "Complete pipeline: Load image file -> detect edges -> optionally save result.
   
   IMPORTANT: First install opticl with: (ql:quickload :opticl)
   
   Options:
     :threshold - Edge detection threshold (default: 50)
     :output-path - Save result to this file
     :max-size - Maximum dimension in pixels (e.g., 800)
     :auto-scale - Auto-scale if > 1M pixels (default: t)
   
   Usage examples:
     (process-image-file \"photo.jpg\")
     (process-image-file \"photo.jpg\" :threshold 70)
     (process-image-file \"photo.jpg\" :max-size 800)
     (process-image-file \"photo.jpg\" :threshold 60 :output-path \"edges.png\")
     (process-image-file \"photo.jpg\" :auto-scale nil)  ; No scaling
   
   Returns: (values grayscale-image edge-image width height)"
  (let* ((grayscale (load-image-file filepath :max-size max-size :auto-scale auto-scale)))
    (when grayscale
      (let* ((height (array-dimension grayscale 0))
             (width (array-dimension grayscale 1))
             (edges (detect-edges grayscale width height threshold)))
        
        (format t "✓ Applied edge detection with threshold ~d~%" threshold)
        
        ;; Save result if output path provided
        (when output-path
          (save-edge-image edges width height output-path))
        
        (values grayscale edges width height)))))

(defun save-edge-image (edge-array width height output-path)
  "Saves edge-detected image to file (PNG format recommended).
   Requires opticl library: (ql:quickload :opticl)"
  (handler-case
      (progn
        ;; Check if opticl is available
        (unless (find-package :opticl)
          (format t "~%ERROR: OPTICL package not found!~%")
          (format t "Please install it first with:~%")
          (format t "  (ql:quickload :opticl)~%~%")
          (error "OPTICL not installed"))
        
        ;; Convert grayscale to RGB for saving
        (let ((make-rgb (intern "MAKE-8-BIT-RGB-IMAGE" :opticl))
              (write-file (intern "WRITE-IMAGE-FILE" :opticl)))
          (let ((rgb-image (funcall make-rgb height width)))
            (dotimes (y height)
              (dotimes (x width)
                (let ((val (round (aref edge-array y x))))
                  (setf (aref rgb-image y x 0) val)
                  (setf (aref rgb-image y x 1) val)
                  (setf (aref rgb-image y x 2) val))))
            
            (funcall write-file output-path rgb-image)
            (format t "✓ Saved edge image to: ~a~%" output-path)
            t)))
    (error (e)
      (format t "~%ERROR: Could not save image!~%")
      (format t "Reason: ~a~%~%" e)
      (format t "Make sure opticl is installed: (ql:quickload :opticl)~%")
      nil)))

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

;; 4. Helper Functions for Testing

(defun create-test-image (width height &optional (pattern 'rectangle))
  "Creates a test image with various patterns for edge detection testing."
  (let ((image (make-array (list height width) :initial-element 0)))
    (case pattern
      (rectangle
       ;; Draw a white rectangle in the center
       (let ((x1 (floor width 4))
             (x2 (floor (* 3 width) 4))
             (y1 (floor height 4))
             (y2 (floor (* 3 height) 4)))
         (dotimes (y height)
           (dotimes (x width)
             (when (and (>= x x1) (<= x x2) (>= y y1) (<= y y2))
               (setf (aref image y x) 255))))))
      (circle
       ;; Draw a white circle in the center
       (let* ((cx (floor width 2))
              (cy (floor height 2))
              (radius (floor (min width height) 4)))
         (dotimes (y height)
           (dotimes (x width)
             (let ((dist (sqrt (+ (expt (- x cx) 2) (expt (- y cy) 2)))))
               (when (<= dist radius)
                 (setf (aref image y x) 255)))))))
      (gradient
       ;; Draw a horizontal gradient
       (dotimes (y height)
         (dotimes (x width)
           (setf (aref image y x) (* (/ x width) 255)))))
      (cross
       ;; Draw a cross pattern
       (let ((mid-x (floor width 2))
             (mid-y (floor height 2))
             (thickness 3))
         (dotimes (y height)
           (dotimes (x width)
             (when (or (and (>= x (- mid-x thickness)) (<= x (+ mid-x thickness)))
                       (and (>= y (- mid-y thickness)) (<= y (+ mid-y thickness))))
               (setf (aref image y x) 255)))))))
    image))

(defun print-image-ascii (image width height &optional (threshold 128))
  "Prints an image as ASCII art. Values above threshold show as '#', below as ' '."
  (dotimes (y height)
    (dotimes (x width)
      (if (> (aref image y x) threshold)
          (format t "#")
          (format t " ")))
    (format t "~%")))

(defun print-image-detailed (image width height)
  "Prints an image with actual pixel values (useful for small images)."
  (dotimes (y height)
    (dotimes (x width)
      (format t "~3d " (round (aref image y x))))
    (format t "~%")))

;; 5. Main Edge Detection Pipeline

(defun detect-edges (image width height &optional (threshold 50))
  "Complete edge detection pipeline: blur -> sobel -> threshold.
   Returns the edge-detected image."
  (let* ((blurred (apply-gaussian-blur image width height))
         (edges (apply-sobel-and-threshold blurred width height threshold)))
    edges))

(defun edge-detection-demo (&optional (width 20) (height 20) (pattern 'rectangle))
  "Runs a complete edge detection demonstration with visualization.
   
   Usage examples:
     (edge-detection-demo)                    ; Rectangle with default size
     (edge-detection-demo 30 30 'circle)      ; Circle pattern
     (edge-detection-demo 25 25 'cross)       ; Cross pattern
     (edge-detection-demo 20 20 'gradient)    ; Gradient pattern
   
   Returns: (values original-image edge-detected-image width height)"
  (format t "~%=== Edge Detection Demo ===~%")
  (format t "Pattern: ~a, Size: ~dx~d~%~%" pattern width height)
  
  (let* ((original (create-test-image width height pattern))
         (edges (detect-edges original width height 50)))
    
    (format t "Original Image:~%")
    (print-image-ascii original width height)
    
    (format t "~%Edge-Detected Image:~%")
    (print-image-ascii edges width height 50)
    
    (format t "~%Done! Edge detection complete.~%")
    (values original edges width height)))

;; 6. Step-by-step demonstration

(defun edge-detection-steps (image width height &optional (threshold 50))
  "Shows each step of the edge detection process with visualization."
  (format t "~%=== Step-by-Step Edge Detection ===~%")
  
  (format t "~%Step 1: Original Image~%")
  (print-image-ascii image width height)
  
  (format t "~%Step 2: After Gaussian Blur~%")
  (let ((blurred (apply-gaussian-blur image width height)))
    (print-image-ascii blurred width height)
    
    (format t "~%Step 3: After Sobel Edge Detection (threshold: ~d)~%" threshold)
    (let ((edges (apply-sobel-and-threshold blurred width height threshold)))
      (print-image-ascii edges width height 50)
      edges)))

;; Example usage in REPL:
;;
;; 1. Load this file:
;;    (load "edge-detection.lisp")
;;
;; 2. Run the demo:
;;    (edge-detection-demo)
;;    (edge-detection-demo 30 30 'circle)
;;    (edge-detection-demo 25 25 'cross)
;;
;; 3. Create custom image:
;;    (defvar my-image (create-test-image 20 20 'rectangle))
;;
;; 4. Apply edge detection:
;;    (defvar edges (detect-edges my-image 20 20 50))
;;
;; 5. Display results:
;;    (print-image-ascii edges 20 20)
;;
;; 6. Step-by-step visualization:
;;    (edge-detection-steps my-image 20 20 50)
;;
;; 7. Try individual steps:
;;    (defvar blurred (apply-gaussian-blur my-image 20 20))
;;    (defvar sobel-result (apply-sobel-and-threshold blurred 20 20 50))
;;    (print-image-ascii sobel-result 20 20)
;;
;; 8. Load and process JPG/PNG images (requires opticl):
;;    First install opticl: (ql:quickload :opticl)
;;
;;    Simple with auto-scaling (recommended for large images):
;;    (process-image-file "photo.jpg" :threshold 50 :output-path "edges.png")
;;
;;    Limit image size to 800 pixels max dimension:
;;    (process-image-file "photo.jpg" :max-size 800 :output-path "edges.png")
;;
;;    Process without scaling (may run out of memory on large images!):
;;    (process-image-file "photo.jpg" :auto-scale nil)
;;
;;    Manual control:
;;    (defvar photo (load-image-file "photo.jpg"))  ; Auto-scales if > 1M pixels
;;    (defvar photo-small (load-image-file "photo.jpg" :max-size 600))
;;    
;;    Get dimensions:
;;    (defvar height (array-dimension photo 0))
;;    (defvar width (array-dimension photo 1))
;;    
;;    Process it:
;;    (defvar photo-edges (detect-edges photo width height 50))
;;    
;;    Save result:
;;    (save-edge-image photo-edges width height "edges-output.png")
