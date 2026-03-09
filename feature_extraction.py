from PIL import Image # Allowed ONLY for decoding the image to RGB [3]
import numpy as np
import matplotlib.pyplot as plt

# define utils
def downscale(input_path, target_size=512):
    # 1. Decode the compressed image into raw RGB data
    img = Image.open(input_path).convert('RGB')
    pixels = img.load()
    orig_width, orig_height = img.size
    
    # Create a new blank 512x512 image array to act as our frame buffer
    out_img = Image.new('RGB', (target_size, target_size))
    out_pixels = out_img.load()
    
    # 2. Calculate the scaling ratios
    # This tells us how many pixels to "skip" in the original image 
    # for every 1 pixel in our new target image.
    x_ratio = orig_width / target_size
    y_ratio = orig_height / target_size
    
    # 3. Hand-crafted downscaling loop (Nearest Neighbor method)
    # We iterate exactly 512x512 times to fill our new frame
    for y in range(target_size):
        for x in range(target_size):
            
            # Map the 512x512 coordinates back to the original image's coordinates
            src_x = int(x * x_ratio)
            src_y = int(y * y_ratio)
            
            # Ensure we don't accidentally pull an index out of bounds
            src_x = min(src_x, orig_width - 1)
            src_y = min(src_y, orig_height - 1)
            
            # Extract the raw RGB values from the original pixel 
            # and map them to the new 512x512 output
            r, g, b = pixels[src_x, src_y] # type: ignore
            out_pixels[x, y] = (r, g, b) # type: ignore
            
    print(f"Successfully downscaled {orig_width}x{orig_height} to 512x512!")
    return out_img

def convolution(image, kernel):
    # Get dimensions of the image and kernel
    image_height, image_width = image.shape
    kernel_height, kernel_width = kernel.shape
    
    # Calculate padding for the image
    pad_height = kernel_height // 2
    pad_width = kernel_width // 2
    
    # Pad the image with zeros on the borders
    padded_image = np.pad(image, ((pad_height, pad_height), (pad_width, pad_width)), mode='constant')
    
    # Prepare an output array to store the convolved result
    output = np.zeros_like(image)
    
    # Perform convolution
    for i in range(image_height):
        for j in range(image_width):
            # Extract the region of interest
            region = padded_image[i:i+kernel_height, j:j+kernel_width]
            # Apply the kernel to the region and sum the result
            output[i, j] = np.sum(region * kernel)
    
    return output

def rgb_to_hsv(rgb_image):
  # Normalize RGB values to [0, 1]
  rgb_normalized = rgb_image / 255.0
  r, g, b = rgb_normalized[:,:,0], rgb_normalized[:,:,1], rgb_normalized[:,:,2]
  
  cmax = np.max(rgb_normalized, axis=2)
  cmin = np.min(rgb_normalized, axis=2)
  delta = cmax - cmin
  
  # Hue calculation
  hue = np.zeros_like(cmax)
  hue[delta != 0] = np.where(cmax[delta != 0] == r[delta != 0], (g[delta != 0] - b[delta != 0]) / delta[delta != 0],
                              np.where(cmax[delta != 0] == g[delta != 0], (b[delta != 0] - r[delta != 0]) / delta[delta != 0] + 2,
                                        (r[delta != 0] - g[delta != 0]) / delta[delta != 0] + 4))
  hue = (hue * 60) % 360
  
  # Saturation calculation
  saturation = np.zeros_like(cmax)
  saturation[cmax != 0] = delta[cmax != 0] / cmax[cmax != 0]
  
  # Value calculation
  value = cmax
  
  return np.stack((hue, saturation, value), axis=2)


# sobel
sobel_x = np.array([[-1, 0, 1],
                    [-2, 0, 2],
                    [-1, 0, 1]])

sobel_y = np.array([[1, 2, 1],
                    [0, 0, 0],
                    [-1, -2, -1]])

# combined xy
def combine_edge_xy(edge_x, edge_y):
    return np.sqrt(edge_x**2 + edge_y**2)
  
def threshold(image, thresh_value):
  # Create a binary image based on the threshold value
  binary_image = np.zeros_like(image)
  binary_image[image >= thresh_value] = 255
  return binary_image

# morphology, connect edges - pure implementation

# dilation
def dilation(image, structure=None):
  if structure is None:
    structure = np.ones((3, 3), dtype=np.uint8)  # Default to a 3x3 square structuring element
  
  # Pad the image to handle borders
  pad_height = structure.shape[0] // 2
  pad_width = structure.shape[1] // 2
  padded_image = np.pad(image, ((pad_height, pad_height), (pad_width, pad_width)), mode='constant')
  
  dilated = np.zeros_like(image)
  
  for i in range(image.shape[0]):
    for j in range(image.shape[1]):
      # Extract the region of interest
      region = padded_image[i:i+structure.shape[0], j:j+structure.shape[1]]
      # If any pixel in the region is set and corresponds to the structuring element, set the output pixel
      if np.any(region * structure):
        dilated[i, j] = 255
  
  return dilated
  
# erosion
def erosion(image, structure=None):
  if structure is None:
      structure = np.ones((3, 3), dtype=np.uint8)  # Default to a 3x3 square structuring element
  
  # Pad the image to handle borders
  pad_height = structure.shape[0] // 2
  pad_width = structure.shape[1] // 2
  padded_image = np.pad(image, ((pad_height, pad_height), (pad_width, pad_width)), mode='constant')
  
  eroded = np.zeros_like(image)
  
  for i in range(image.shape[0]):
    for j in range(image.shape[1]):
      # Extract the region of interest
      region = padded_image[i:i+structure.shape[0], j:j+structure.shape[1]]
      # If all pixels in the region that correspond to the structuring element are set, set the output pixel
      if np.all(region * structure):
        eroded[i, j] = 255
  
  return eroded

# closing
def morphological_closing(image, structure=None):
  if structure is None:
    structure = np.ones((3, 3), dtype=np.uint8)  # Default to a 3x3 square structuring element
  
  # Dilation
  dilated = dilation(image, structure)
  
  # Erosion
  closed = erosion(dilated, structure)
  
  return closed

# opening
def morphological_opening(image, structure=None):
  if structure is None:
    structure = np.ones((3, 3), dtype=np.uint8)  # Default to a 3x3 square structuring element
  
  # Erosion
  eroded = erosion(image, structure)
  
  # Dilation
  opened = dilation(eroded, structure)
  
  return opened

def extract_segments(edges, threshold=50):
    h, w = edges.shape
    visited = np.zeros((h, w), dtype=bool)

    segments = []

    for y in range(h):
        for x in range(w):

            if edges[y, x] > threshold and not visited[y, x]:

                stack = [(y, x)]
                segment = []

                visited[y, x] = True

                while stack:
                    cy, cx = stack.pop()
                    segment.append((cy, cx))

                    for ny, nx in [
                        (cy-1,cx),(cy+1,cx),(cy,cx-1),(cy,cx+1),
                        (cy-1,cx-1),(cy-1,cx+1),(cy+1,cx-1),(cy+1,cx+1)
                    ]:
                        if 0 <= ny < h and 0 <= nx < w:
                            if edges[ny,nx] > threshold and not visited[ny,nx]:
                                visited[ny,nx] = True
                                stack.append((ny,nx))

                segments.append(segment)

    return segments

## -- ##

img = np.array(downscale("frame1.jpg", 128))
