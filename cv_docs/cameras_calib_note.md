# Cam Calib Notes

-----

[TOC]

## When To Calib
Your camera might be out of calibration if you had the following symptoms:
* Reduced depth density on objects in the operating range (might still get some
depth).
* Flat surfaces look "wobbly", i.e. there is more deviation from the flatness than usual.
* Measuring the physical distance to objects are not within expected range.


## Calibration Best Practices

* Choose the right size calibration target
* Perform calibration at the approximate **working distance (WD)** of your final application
* Use good lighting
  - normal lighting conditions from general office lighting (around 200 LUX) to more
  brighter lighting with additional floor lamp (around 1000 LUX)
  - The lighting on the chart should be generally uniform without hotspots
* Collect images from different areas and tilts
* Have enough observations (at least 6 observations)
* Consider using **CharuCo boards**
* Calibration is only as accurate as the calibration target used
* Proper mounting of calibration target and camera
* Remove bad observations


## Calibration Patterns

### Pattern size

As a rule of thumb, the calibration plate should have an area of at least half the available pixel area when observed frontally

### Pattern type
* Checkerboard
* Circle grids
* CharuCo targets
* AprilTag

## Camera Models


## Calibration Quality Check

### Quick Check
* Point the camera to a flat surface such as a wall about 1 to 2 meters away (3 to 6 feet) and avoid black surfaces. Visually inspect the depth image display of the wall. A lot of black dots or holes on the image is an indication of the camera being out of calibration.

### Accuracy Check
This procedure should be used to check the accuracy of the camera.

* Calibrate the camera with the ATAN model and make sure you have a very low reprojection error (~0.1px).

* Expect accuracy within 2% at @2 meters
  * Place the camera in parallel to a flat wall and exactly two meter (2000 mm) away. Once the camera is placed in its position, Use Intel® RealSenseTMViewer or Depth Quality Tool to measure the absolute distance. For a flat surface at a distance of 2 meter the absolute distance should be within 2% or better at 2 meter (2000mm). If the distance is not within the defined range, then the camera needs to be calibrated.
