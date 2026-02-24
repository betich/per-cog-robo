(load "edge-detection.lisp")
(ql:quickload :opticl :silent t)

(format t "~%Testing with frame1.jpg (4096x3072 pixels)~%~%")
(format t "This will auto-scale the image to prevent memory issues.~%~%")

;; Process with auto-scaling and save result
(process-image-file "frame1.jpg" :threshold 50 :output-path "edges-output.png")

(format t "~%✓ Success! Check edges-output.png~%")
