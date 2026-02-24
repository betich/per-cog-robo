# Perception of Cognitive Robots

This project is part of the **Perception of Cognitive Robots** class at the **International School of Engineering (ISE)**, Chulalongkorn University.

## Project Description

This repository contains implementations of various perception and robotics algorithms in Common Lisp, including:

- Edge detection algorithms
- Kalman filtering for state estimation

## Setup Instructions

### Prerequisites

You need to have a Common Lisp implementation installed. We recommend:

- **SBCL (Steel Bank Common Lisp)** - recommended for most users
- **CCL (Clozure Common Lisp)** - alternative option
- **CLISP** - another alternative

### Installing Common Lisp

#### macOS

```bash
# Using Homebrew
brew install sbcl
```

#### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install sbcl
```

#### Windows

Download SBCL from [http://www.sbcl.org/platform-table.html](http://www.sbcl.org/platform-table.html)

### Installing Dependencies

This project requires **Quicklisp** (Common Lisp package manager) and several libraries.

#### 1. Install Quicklisp

```bash
# Download Quicklisp
curl -O https://beta.quicklisp.org/quicklisp.lisp

# Install it
sbcl --load quicklisp.lisp
```

In the SBCL REPL that opens:

```lisp
(quicklisp-quickstart:install)
(ql:add-to-init-file)  ; Automatically loads Quicklisp on SBCL startup
(quit)
```

#### 2. Install Required Packages

Start SBCL and install the dependencies:

```bash
sbcl
```

In the REPL:

```lisp
;; Install GSLL (GNU Scientific Library for kalman.lisp)
(ql:quickload :gsll)

;; Install opticl (optional - for loading JPG/PNG images in edge-detection.lisp)
(ql:quickload :opticl)

(quit)
```

**Note about SKETCH:** The `kalman.lisp` file uses SKETCH for graphical visualization, which requires `c2ffi` and SDL2. This can be challenging to set up on some systems. See the [Troubleshooting](#troubleshooting) section below for solutions.

**Note about Image Loading:** The edge detection code works without any dependencies using built-in test patterns. To load actual JPG/PNG image files, you'll need the `opticl` library (see examples below).

### Running the Project

1. **Clone the repository**

   ```bash
   git clone <repository-url>
   cd per-cog-robo
   ```

2. **Run Lisp files directly**

   ```bash
   # Edge detection (no dependencies required)
   sbcl --load edge-detection.lisp

   # Kalman filter (requires SKETCH and GSLL)
   sbcl --eval "(ql:quickload '(:sketch :gsll))" --load kalman.lisp
   ```

3. **Interactive REPL (Read-Eval-Print Loop)**

   ```bash
   # Start SBCL
   sbcl
   ```

   Then in the REPL:

   ```lisp
   ;; Load a file
   (load "edge-detection.lisp")
   (load "kalman.lisp")

   ;; Call functions defined in the files
   ;; (function-name arguments)
   ```

4. **Exit the REPL**
   ```lisp
   (quit)
   ```

### Processing Images with opticl

```
(process-image-file "frame1.jpg" :threshold 50 :output-path "edges-output.png")
```

## Using Edge Detection (REPL Examples)

The `edge-detection.lisp` file includes built-in demo functions for easy testing. Here's how to use them:

### Quick Demo

```bash
# Start SBCL
sbcl

# In the REPL:
```

```lisp
;; Load the edge detection file
(load "edge-detection.lisp")

;; Run the demo with default settings (20x20 rectangle)
(edge-detection-demo)

;; Try different patterns and sizes:
(edge-detection-demo 30 30 'circle)
(edge-detection-demo 25 25 'cross)
(edge-detection-demo 20 20 'gradient)
```

### Step-by-Step Processing

```lisp
;; Create a test image
(defvar my-image (create-test-image 20 20 'rectangle))

;; Apply complete edge detection pipeline
(defvar edges (detect-edges my-image 20 20 50))

;; Display the result as ASCII art
(print-image-ascii edges 20 20)

;; See step-by-step visualization
(edge-detection-steps my-image 20 20 50)
```

### Working with Individual Functions

```lisp
;; 1. Create test image (20x20 pixels)
(defvar img (create-test-image 20 20 'circle))

;; 2. Display original image
(print-image-ascii img 20 20)

;; 3. Apply Gaussian blur
(defvar blurred (apply-gaussian-blur img 20 20))

;; 4. Apply Sobel edge detection with threshold of 50
(defvar sobel-result (apply-sobel-and-threshold blurred 20 20 50))

;; 5. Display final edge-detected image
(print-image-ascii sobel-result 20 20)

;; 6. Display with actual pixel values (for small images)
(print-image-detailed sobel-result 20 20)
```

### Available Test Patterns

- `'rectangle` - White rectangle in center (default)
- `'circle` - White circle in center
- `'cross` - Cross/plus sign pattern
- `'gradient` - Horizontal gradient

### Adjusting Parameters

```lisp
;; Higher threshold = fewer edges detected
(edge-detection-demo 25 25 'circle)  ; Uses default threshold of 50

;; Custom edge detection with different threshold
(defvar img (create-test-image 30 30 'cross))
(defvar edges-low (detect-edges img 30 30 30))    ; More edges
(defvar edges-high (detect-edges img 30 30 100))  ; Fewer edges

;; Compare results
(format t "Low threshold (30):~%")
(print-image-ascii edges-low 30 30)
(format t "~%High threshold (100):~%")
(print-image-ascii edges-high 30 30)
```

### Loading Real Images (JPG/PNG)

**⚠️ IMPORTANT: Install opticl first!**

If you see the error `Package OPTICL does not exist`, you need to install it:

```bash
# Start SBCL
sbcl
```

```lisp
;; Install opticl (only needed once)
(ql:quickload :opticl)

;; Quit and restart
(quit)
```

**Quick install script:**

```bash
# Or use the helper script
sbcl --load install-opticl.lisp --eval "(quit)"
```

Once opticl is installed, use these functions:

```lisp
;; Load your edge detection code
(load "edge-detection.lisp")

;; RECOMMENDED: Auto-scaling for large images (prevents memory errors)
(process-image-file "photo.jpg" :threshold 50 :output-path "edges.png")

;; Limit image size to 800 pixels (fastest, good for 4K+ images)
(process-image-file "photo.jpg" :max-size 800 :output-path "edges.png")

;; With custom threshold
(process-image-file "photo.jpg" :threshold 70 :max-size 1000)

;; Disable auto-scaling (USE WITH CAUTION - may run out of memory!)
(process-image-file "photo.jpg" :auto-scale nil)

;; Manual processing for more control:
;; 1. Load the image (auto-scales if > 1 million pixels)
(defvar photo (load-image-file "photo.jpg"))

;; Load with specific max size
(defvar photo-small (load-image-file "photo.jpg" :max-size 600))

;; 2. Get dimensions
(defvar height (array-dimension photo 0))
(defvar width (array-dimension photo 1))

;; 3. Apply edge detection
(defvar photo-edges (detect-edges photo width height 50))

;; 4. Save result
(save-edge-image photo-edges width height "output-edges.png")
```

**Supported formats:** JPG, JPEG, PNG, BMP, PNM, GIF, TIFF

**Example workflow:**

```bash
# Place a photo in your project directory, then:
sbcl
```

```lisp
(load "edge-detection.lisp")
(ql:quickload :opticl)
(process-image-file "myPhoto.jpg" :threshold 50 :output-path "myPhoto-edges.png")
(quit)
```

**⚠️ Memory Issues with Large Images:**

If you see `Heap exhausted` errors with large images (like 4K photos):

```lisp
;; Solution 1: Use max-size to limit dimensions (RECOMMENDED)
(process-image-file "4k-photo.jpg" :max-size 800 :output-path "edges.png")

;; Solution 2: Images > 1M pixels are auto-scaled by default
(process-image-file "large-photo.jpg")  ; Automatically scales down

;; Solution 3: Increase SBCL memory (advanced)
;; Start SBCL with more memory:
;; sbcl --dynamic-space-size 4096
```

**Performance tips:**

- Images are automatically scaled if > 1 million pixels
- For 4K+ images (4096x3072), use `:max-size 800` or `:max-size 1000`
- Smaller images = faster processing and less memory
- Edge detection quality remains good even at reduced sizes

### Exit REPL

```lisp
(quit)
```

## Quick Reference - Common REPL Commands

### Getting Started

```bash
# Start SBCL
sbcl

# Load the edge detection file
(load "edge-detection.lisp")

# Run quick demo
(edge-detection-demo)

# Exit when done
(quit)
```

### Available Demo Functions

| Function                                     | Description                | Example                               |
| -------------------------------------------- | -------------------------- | ------------------------------------- |
| `(edge-detection-demo)`                      | Quick demo with defaults   | `(edge-detection-demo)`               |
| `(edge-detection-demo width height pattern)` | Custom size and pattern    | `(edge-detection-demo 30 30 'circle)` |
| `(create-test-image w h pattern)`            | Create test image          | `(create-test-image 20 20 'cross)`    |
| `(detect-edges img w h threshold)`           | Apply edge detection       | `(detect-edges img 20 20 50)`         |
| `(print-image-ascii img w h)`                | Display as ASCII art       | `(print-image-ascii img 20 20)`       |
| `(edge-detection-steps img w h)`             | Step-by-step visualization | `(edge-detection-steps img 20 20 50)` |

### Image File Functions (requires opticl)

First install: `(ql:quickload :opticl)`

| Function                                                            | Description            | Example                                                                   |
| ------------------------------------------------------------------- | ---------------------- | ------------------------------------------------------------------------- |
| `(load-image-file path)`                                            | Load with auto-scaling | `(load-image-file "photo.jpg")`                                           |
| `(load-image-file path :max-size N)`                                | Load with size limit   | `(load-image-file "photo.jpg" :max-size 800)`                             |
| `(process-image-file path :threshold N :max-size M :output-path P)` | Complete pipeline      | `(process-image-file "photo.jpg" :max-size 800 :output-path "edges.png")` |
| `(save-edge-image img w h path)`                                    | Save result to file    | `(save-edge-image edges 100 100 "out.png")`                               |

**Key parameters:**

- `:threshold` - Edge sensitivity (30-100, default: 50)
- `:max-size` - Max dimension in pixels (recommended: 600-1000 for large images)
- `:auto-scale` - Auto-scale if > 1M pixels (default: t)
- `:output-path` - Where to save the result

### Pattern Options

- `'rectangle` - Rectangle in center
- `'circle` - Circle in center
- `'cross` - Cross/plus pattern
- `'gradient` - Horizontal gradient

### One-Liner Demos

```bash
# Rectangle pattern
sbcl --noinform --eval "(load \"edge-detection.lisp\")" --eval "(edge-detection-demo)" --eval "(quit)"

# Circle pattern
sbcl --noinform --eval "(load \"edge-detection.lisp\")" --eval "(edge-detection-demo 25 25 'circle)" --eval "(quit)"

# Cross pattern
sbcl --noinform --eval "(load \"edge-detection.lisp\")" --eval "(edge-detection-demo 25 25 'cross)" --eval "(quit)"
```

### Development Environment (Optional)

For a better development experience, consider using:

- **Emacs with SLIME** (Superior Lisp Interaction Mode for Emacs)
  ```bash
  # Install via Homebrew on macOS
  brew install emacs
  ```
- **Visual Studio Code with Alive extension** - for VS Code users
- **Portacle** - A portable, all-in-one Lisp development environment

## Troubleshooting

### SKETCH Package Issues (kalman.lisp)

If you encounter errors like `"SKETCH" does not designate any package` or `Couldn't execute "c2ffi"`, this is because SKETCH requires additional system dependencies that can be difficult to install:

**The Issue:**

- SKETCH requires SDL2 (usually available via Homebrew)
- SKETCH also requires `c2ffi` (C to FFI generator), which is not easily available on modern systems

**Solutions:**

1. **For macOS - Try Roswell:**

   ```bash
   # Install Roswell (alternative Lisp environment manager)
   brew install roswell

   # Use Roswell's SBCL
   ros install sbcl
   ros use sbcl
   ```

2. **Use edge-detection.lisp instead:**
   The edge detection implementation works without any complex dependencies:

   ```bash
   sbcl --load edge-detection.lisp
   ```

3. **Run kalman filter without graphics:**
   You can modify kalman.lisp to remove SKETCH dependencies if you only need the calculation logic

4. **Manual c2ffi installation (advanced):**
   Visit [c2ffi GitHub](https://github.com/rpav/c2ffi) for source installation instructions

**For class purposes:** If you only need to demonstrate the algorithms without visualization, `edge-detection.lisp` runs perfectly without these dependencies.

## Project Files

- `edge-detection.lisp` - Edge detection algorithms (no dependencies required)
- `kalman.lisp` - Kalman filter with visualization (requires SKETCH and GSLL)
- `README.md` - This file

## License

This project is for educational purposes as part of coursework at Chulalongkorn University.
