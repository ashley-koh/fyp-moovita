# Basic Lane Tracking Theory & Example

Table of Contents:

1. [Getting Started](#getting-started)
2. [How It All Works](#how-it-all-works)
    * [Importing Image or Video](#importing-image-or-video)
    * [Cannying](#cannying)

## Getting Started

### Prerequisites

You first need to have OpenCV and NumPy installed to run the script. So Run:

`pip install opencv-python numpy matplotlib`

### Installation

Clone this git repository

`git clone https://github.com/ashley-koh/fyp-moovita.git`

And cd into this folder

`cd basic-lane-detector`

To run the script, run

`python lanes.py`

Make sure `test_image.jpg` and `test2.mp4` are in the same folder as the script

## How It All Works

There are also some comments within the script itself that allow you to understand what is happening.

### Importing Image or Video

If you want the CV to run on the test image, use:

```
image = cv2.read('test_image.jpg')
# Rest of Code
```

Else, if you want to run it on the test video, use:

```
cap = cv2.VideoCapture("test2.mp4")
while(cap.isOpened()):
    # Rest of Code

cap.release()
cv2.destroyAllWindows()
```

OpenCV will convert the read image/frame into a NumPy Array signifying the RGB value of each pixel.

### Cannying

#### Method

```
def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) # Conver to Gray Scale
    blur = cv2.GaussianBlur(gray, (5, 5,), 0) # Smoothen out received image
    canny = cv2.Canny(blur, 50, 150) # Detect Edges
    return canny
```

#### Steps

1. Convert input image into Gray Scale
    * This will convert the 3 dimensional NumPy array into 1 dimension since all values of RGB in gray scale are the same.

2. Apply Gaussian Blur to reduce image noise
    * By using the grey scale value of pixels around a chosen pixel, Gaussian Blur will either lighten or darken the chosen pixel so that the colour gradient will not be as draastic. Hence, making the image appear smoother. We do this with a 5x5 grid with the current pixel at the center.
    
    * Here is an example:
    
    ![OpenCV smoothening example](https://docs.opencv.org/3.1.0/filter.jpg)

3. Apply Cannying
    * Cannying will trace an edge when it has a large change in colour gradient. Just like the edges of the white markings on dark coloured roads. We also have to provide a threshold to the canny method so it filters out unwanted edges that have small changes in gradient.

    * Here is example:

    ![OpenCV cannying example](http://scikit-image.org/docs/dev/_images/sphx_glr_plot_canny_001.png)
