#!/usr/bin/env python
"""
Image matching and graph visualization tool.
Matches images using ORB features and creates a graph visualization of the matches.
"""

import argparse
import sys
from typing import List, Tuple, Any
from libccv.common import imtools as imt
import numpy as np
from PIL import Image
import cv2 as cv
import pydot

# Global variable to store image list
imlist: List[str] = []


def draw_keypoints(img0: np.ndarray, img1: np.ndarray, kp0: Any, kp1: Any) -> None:
    """
    Draw keypoints on two images and display them side by side.

    Args:
        img0: First image
        img1: Second image
        kp0: Keypoints for first image
        kp1: Keypoints for second image
    """
    outimg0 = cv.drawKeypoints(img0, keypoints=kp0, outImage=None)
    outimg1 = cv.drawKeypoints(img1, keypoints=kp1, outImage=None)
    outimg = np.hstack([outimg0, outimg1])
    cv.imshow("Key Points", outimg)
    cv.waitKey(0)
    cv.destroyAllWindows()


def draw_match(
    img0: np.ndarray, img1: np.ndarray, kp0: Any, kp1: Any, match: Any
) -> None:
    """
    Draw matches between two images and display the result.

    Args:
        img0: First image
        img1: Second image
        kp0: Keypoints for first image
        kp1: Keypoints for second image
        match: Matches between keypoints
    """
    outimage = cv.drawMatches(img0, kp0, img1, kp1, match, outImg=None)
    cv.imshow("Match Result", outimage)
    cv.waitKey(0)
    cv.destroyAllWindows()


def get_good_matches(matches: Any) -> List[Any]:
    """
    Filter good matches based on distance thresholds.

    When the distance between descriptors is greater than twice the minimum distance,
    the match is considered incorrect. However, sometimes the minimum distance can
    be very small, so we set a lower limit of 10 as a threshold.

    Args:
        matches: List of matches

    Returns:
        List of good matches
    """
    if len(matches) == 0:
        return []

    # Calculate minimum and maximum distances
    min_distance = matches[0].distance
    max_distance = matches[0].distance
    for match in matches:
        if match.distance < min_distance:
            min_distance = match.distance
        if match.distance > max_distance:
            max_distance = match.distance

    # Filter good matches
    good_match = []
    for match in matches:
        if match.distance <= max(2 * min_distance, 10):
            good_match.append(match)
    return good_match


def matches_graphviz(
    match_scores: np.ndarray,
    out_img: str = "img_match_graph.png",
    threshold: float = 30.0,
) -> None:
    """
    Create a graph visualization of image matches.

    Args:
        match_scores: Matrix of match scores between images
        out_img: Output image path
        threshold: Minimum match score threshold for creating edges
    """
    global imlist

    g = pydot.Dot(graph_type="graph")
    n = len(imlist)

    for i in range(n):
        for j in range(i + 1, n):
            if match_scores[i, j] > threshold:
                for x in [i, j]:
                    pil_im = Image.open(imlist[x])
                    pil_im.thumbnail((128, 128))
                    # Use a more descriptive filename
                    im_path = f"/tmp/match_graph_img_{x}.png"
                    pil_im.save(im_path)
                    g.add_node(
                        pydot.Node(
                            str(x),
                            fontcolor="transparent",
                            shape="rectangle",
                            image=im_path,
                        )
                    )
                g.add_edge(pydot.Edge(str(i), str(j)))

    g.write_png(out_img)
    print(f"Graph visualization saved to {out_img}")


def main() -> None:
    """
    Main function to match images and create a graph visualization.
    """
    parser = argparse.ArgumentParser(
        description="Match images using ORB features and create a graph visualization"
    )
    parser.add_argument(
        "-d", "--img_dir", required=True, type=str, help="input image directory"
    )
    parser.add_argument(
        "-e", "--img_ext", default="png", help="image extension (without dot)"
    )
    parser.add_argument(
        "-o", "--output", default="img_match_graph.png", help="output graph image path"
    )
    parser.add_argument(
        "-t",
        "--threshold",
        default=30.0,
        type=float,
        help="match score threshold for graph edges",
    )
    args = parser.parse_args()

    img_dir: str = args.img_dir
    img_ext: str = args.img_ext
    output_path: str = args.output
    threshold: float = args.threshold

    global imlist
    imlist = imt.get_imlist(img_dir, "." + img_ext)
    if len(imlist) == 0:
        print("ERROR: No images found in the specified directory!")
        sys.exit(1)

    print(f"Found {len(imlist)} images")

    n = len(imlist)

    # Create ORB detector and matcher
    orb = cv.ORB_create()
    bf = cv.BFMatcher(cv.NORM_HAMMING)

    # Initialize match scores matrix
    match_scores = np.zeros((n, n))

    # Process each pair of images
    for i in range(n):
        for j in range(i, n):
            # Load images
            img0 = cv.imread(imlist[i])
            img1 = cv.imread(imlist[j])

            # Check if images were loaded successfully
            if img0 is None or img1 is None:
                print(f"Warning: Could not load image(s) for pair ({i}, {j})")
                continue

            # Detect keypoints
            kp0 = orb.detect(img0)
            kp1 = orb.detect(img1)

            # Compute descriptors
            kp0, des0 = orb.compute(img0, kp0)
            kp1, des1 = orb.compute(img1, kp1)

            # Match descriptors
            if (
                des0 is not None
                and des1 is not None
                and len(des0) > 0
                and len(des1) > 0
            ):
                matches = bf.match(des0, des1)
                good_matches = get_good_matches(matches)
                match_scores[i, j] = len(good_matches)
            else:
                match_scores[i, j] = 0
                print(f"Warning: No descriptors found for pair ({i}, {j})")

    # Make the matrix symmetric
    for i in range(n):
        for j in range(i + 1, n):
            match_scores[j, i] = match_scores[i, j]

    print("Match scores matrix:")
    print(match_scores)

    # Create graph visualization
    matches_graphviz(match_scores, output_path, threshold)


if __name__ == "__main__":
    main()
