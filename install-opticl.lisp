;; Helper script to install opticl for image processing
;; Run this file to install the required library

(format t "~%=== Installing OPTICL for Image Processing ===~%~%")

(format t "Checking if Quicklisp is available...~%")

(if (find-package :quicklisp)
    (progn
      (format t "✓ Quicklisp found!~%~%")
      (format t "Installing opticl...~%")
      (ql:quickload :opticl)
      (format t "~%✓ Installation complete!~%~%")
      (format t "You can now use image loading functions:~%")
      (format t "  (load \"edge-detection.lisp\")~%")
      (format t "  (process-image-file \"photo.jpg\")~%~%"))
    (progn
      (format t "✗ Quicklisp not found!~%~%")
      (format t "Please install Quicklisp first:~%")
      (format t "1. Download: curl -O https://beta.quicklisp.org/quicklisp.lisp~%")
      (format t "2. Install: sbcl --load quicklisp.lisp~%")
      (format t "3. In SBCL: (quicklisp-quickstart:install)~%")
      (format t "4. Then: (ql:add-to-init-file)~%~%")))
