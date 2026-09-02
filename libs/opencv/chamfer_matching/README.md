# Canny edge chamfer matching

This example uses the first image as a template and searches for it in the second image. It runs Canny edge detection,
builds an L2 distance map from the search-image edges, and minimizes the mean distance sampled at the template edge
points. A lower chamfer score is a better match.

Build and run:

```bash
cmake -S libs/opencv -B build/opencv_examples
cmake --build build/opencv_examples --target chamfer_matching
./build/opencv_examples/chamfer_matching/chamfer_matching template.png scene.png result.png 50 150
```

The output image contains the search-image edges and a red rectangle around the best match. The program also prints
the top-left match position and the mean chamfer distance in pixels.
