import logging as log
import time
import numpy as np
import cv2
import sys
import os
import signal
from rknnlite.api import RKNNLite


# Jet colormap for visualization.
myjet = np.array(
    [
        [0.0, 0.0, 0.5],
        [0.0, 0.0, 0.99910873],
        [0.0, 0.37843137, 1.0],
        [0.0, 0.83333333, 1.0],
        [0.30044276, 1.0, 0.66729918],
        [0.66729918, 1.0, 0.30044276],
        [1.0, 0.90123457, 0.0],
        [1.0, 0.48002905, 0.0],
        [0.99910873, 0.07334786, 0.0],
        [0.5, 0.0, 0.0],
    ]
)


H, W = 480, 640
cell = 8
border_remove = 4
nms_dist = 4
conf_thresh = 0.015
# Font parameters for visualizaton.
font = cv2.FONT_HERSHEY_DUPLEX
font_clr = (255, 255, 255)
font_pt = (4, 12)
font_sc = 0.4


# self
def nms_fast(in_corners, H, W, dist_thresh):
    """
    Run a faster approximate Non-Max-Suppression on numpy corners shaped:
      3xN [x_i,y_i,conf_i]^T

    Algo summary: Create a grid sized HxW. Assign each corner location a 1, rest
    are zeros. Iterate through all the 1's and convert them either to -1 or 0.
    Suppress points by setting nearby values to 0.

    Grid Value Legend:
    -1 : Kept.
     0 : Empty or suppressed.
     1 : To be processed (converted to either kept or supressed).

    NOTE: The NMS first rounds points to integers, so NMS distance might not
    be exactly dist_thresh. It also assumes points are within image boundaries.

    Inputs
      in_corners - 3xN numpy array with corners [x_i, y_i, confidence_i]^T.
      H - Image height.
      W - Image width.
      dist_thresh - Distance to suppress, measured as an infinty norm distance.
    Returns
      nmsed_corners - 3xN numpy matrix with surviving corners.
      nmsed_inds - N length numpy vector with surviving corner indices.
    """
    grid = np.zeros((H, W)).astype(int)  # Track NMS data.
    inds = np.zeros((H, W)).astype(int)  # Store indices of points.
    # Sort by confidence and round to nearest int.
    inds1 = np.argsort(-in_corners[2, :])
    corners = in_corners[:, inds1]
    rcorners = corners[:2, :].round().astype(int)  # Rounded corners.
    # Check for edge case of 0 or 1 corners.
    if rcorners.shape[1] == 0:
        return np.zeros((3, 0)).astype(int), np.zeros(0).astype(int)
    if rcorners.shape[1] == 1:
        out = np.vstack((rcorners, in_corners[2])).reshape(3, 1)
        return out, np.zeros((1)).astype(int)
    # Initialize the grid.
    for i, rc in enumerate(rcorners.T):
        grid[rcorners[1, i], rcorners[0, i]] = 1
        inds[rcorners[1, i], rcorners[0, i]] = i
    # Pad the border of the grid, so that we can NMS points near the border.
    pad = dist_thresh
    grid = np.pad(grid, ((pad, pad), (pad, pad)), mode="constant")
    # Iterate through points, highest to lowest conf, suppress neighborhood.
    count = 0
    for i, rc in enumerate(rcorners.T):
        # Account for top and left padding.
        pt = (rc[0] + pad, rc[1] + pad)
        if grid[pt[1], pt[0]] == 1:  # If not yet suppressed.
            grid[pt[1] - pad : pt[1] + pad + 1, pt[0] - pad : pt[0] + pad + 1] = 0
            grid[pt[1], pt[0]] = -1
            count += 1
    # Get all surviving -1's and return sorted array of remaining corners.
    keepy, keepx = np.where(grid == -1)
    keepy, keepx = keepy - pad, keepx - pad
    inds_keep = inds[keepy, keepx]
    out = corners[:, inds_keep]
    values = out[-1, :]
    inds2 = np.argsort(-values)
    out = out[:, inds2]
    out_inds = inds1[inds_keep[inds2]]
    return out, out_inds


def load_images_from_folder(folder):
    images = []
    rknn = load_model()
    for filename in os.listdir(folder):
        # 仅处理图像文件
        if filename.endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif")):
            img_path = os.path.join(folder, filename)
            image_raw = cv2.imread(img_path, 0)  # 读取为灰度图像
            image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)  # 读取为灰度图像
            interp = cv2.INTER_AREA
            grayim = cv2.resize(image_raw, (640, 480), interpolation=interp)
            grayim = grayim.astype("float32") / 255.0
            # image_resized = cv2.resize(image, (160, 120))  # 根据模型要求调整大小w，h

            # # 规范化图像
            # image_float = image_resized.astype(np.float32) / 255.0  # 将像素值从0-255范围转换为0-1范围

            # # 添加 batch 维度，并调整维度顺序
            image_input = np.expand_dims(grayim, axis=0)  # 变为 [1, height, width]
            image_input = np.expand_dims(
                image_input, axis=0
            )  # 变为 [1, 1, height, width]
            image_input = np.transpose(
                image_input, (0, 2, 3, 1)
            )  # 调整为 [batch, height, width, channels]

            s = time.time()
            output = rknn.inference(inputs=[image_input])
            print("main func, get objs use {} ms. ".format((time.time() - s) * 1000))
            print(f"img_reslt:{len(output)}")

            # # 处理输出 (假设输出包括关键点的坐标和描述子)
            keypoints = output[0]  # 这里假定第一个输出为关键点
            descriptors = output[1]  # 假定第二个输出为描述子
            print(keypoints.shape)
            print(descriptors.shape)
            print(type(keypoints))
            semi = keypoints.squeeze()  # 去掉维度为1的

            # --- Process points.
            dense = np.exp(semi)  # Softmax.
            dense = dense / (np.sum(dense, axis=0) + 0.00001)  # Should sum to 1.
            # Remove dustbin.
            nodust = dense[:-1, :, :]
            # Reshape to get full resolution heatmap.
            Hc = int(H / cell)
            Wc = int(W / cell)
            nodust = nodust.transpose(1, 2, 0)
            heatmap = np.reshape(nodust, [Hc, Wc, cell, cell])
            heatmap = np.transpose(heatmap, [0, 2, 1, 3])
            heatmap = np.reshape(heatmap, [Hc * cell, Wc * cell])
            xs, ys = np.where(heatmap >= conf_thresh)  # Confidence threshold.
            if len(xs) == 0:
                return np.zeros((3, 0)), None, None
            pts = np.zeros((3, len(xs)))  # Populate point data sized 3xN.
            pts[0, :] = ys
            pts[1, :] = xs
            pts[2, :] = heatmap[xs, ys]
            pts, _ = nms_fast(pts, H, W, dist_thresh=nms_dist)  # Apply NMS.
            inds = np.argsort(pts[2, :])
            pts = pts[:, inds[::-1]]  # Sort by confidence.
            # Remove points along border.
            bord = border_remove
            toremoveW = np.logical_or(pts[0, :] < bord, pts[0, :] >= (W - bord))
            toremoveH = np.logical_or(pts[1, :] < bord, pts[1, :] >= (H - bord))
            toremove = np.logical_or(toremoveW, toremoveH)
            pts = pts[:, ~toremove]
            if heatmap is not None:
                min_conf = 0.001
                heatmap[heatmap < min_conf] = min_conf
                heatmap = -np.log(heatmap)
                heatmap = (heatmap - heatmap.min()) / (
                    heatmap.max() - heatmap.min() + 0.00001
                )
                out3 = myjet[np.round(np.clip(heatmap * 10, 0, 9)).astype("int"), :]
                out3 = (out3 * 255).astype("uint8")
                cv2.putText(
                    out3,
                    "Raw Point Confidences",
                    font_pt,
                    font,
                    font_sc,
                    font_clr,
                    lineType=16,
                )

            # Extra output -- Show current point detections.
            out2 = (np.dstack((grayim, grayim, grayim)) * 255.0).astype("uint8")
            for pt in pts.T:
                pt1 = (int(round(pt[0])), int(round(pt[1])))
                cv2.circle(out2, pt1, 1, (0, 255, 0), -1, lineType=16)
            cv2.putText(
                out2,
                "Raw Point Detections",
                font_pt,
                font,
                font_sc,
                font_clr,
                lineType=16,
            )

            out = np.hstack((out2, out3))
            # --- Process descriptor.
            # D = coarse_desc.shape[1]
            # if pts.shape[1] == 0:
            # desc = np.zeros((D, 0))
            # else:
            # # Interpolate into descriptor map using 2D point locations.
            # samp_pts = torch.from_numpy(pts[:2, :].copy())
            # samp_pts[0, :] = (samp_pts[0, :] / (float(W)/2.)) - 1.
            # samp_pts[1, :] = (samp_pts[1, :] / (float(H)/2.)) - 1.
            # samp_pts = samp_pts.transpose(0, 1).contiguous()
            # samp_pts = samp_pts.view(1, 1, -1, 2)
            # samp_pts = samp_pts.float()
            # if self.cuda:
            # samp_pts = samp_pts.cuda()
            # desc = torch.nn.functional.grid_sample(coarse_desc, samp_pts)
            # desc = desc.data.cpu().numpy().reshape(D, -1)
            # desc /= np.linalg.norm(desc, axis=0)[np.newaxis, :]
            # return pts, desc, heatmap
            # print("Descriptors:", descriptors)
            # # 将关键点绘制到原始图像上
            # 假设我们提取关键点的方式是寻找每个关键点的最大值
            # 找到关键点的索引
            """
            keypoint_indices = np.argwhere(keypoints > 0.015)  # threshold 是一个合适的值来判断关键点  
            print(f"Extracted keypoints: {keypoint_indices.shape[0]}")  

            #image_resized = np.zeros((480, 640, 3), dtype=np.uint8)  # 创建一个空图像用于展示  

            for idx in keypoint_indices:  
                x, y = idx[0], idx[1]  # 假设 idx 是 (y, x) 格式  
                
                # 绘制关键点  
                #cv2.circle(image_raw, (x, y), radius=5, color=(0, 255, 0), thickness=-1)  # 绘制绿色圆圈  

                # 取描述子（如果描述子与关键点数量匹配且有条件）  
                if x < descriptors.shape[1] and y < descriptors.shape[2]:  # 确保索引有效  
                    descriptor_vector = descriptors[0, :, y, x]  # 假设描述子的维度是 (1, D, H, W)  
                    angle = np.arctan2(descriptor_vector[1], descriptor_vector[0])  # 计算方向角  
                    length = 10  # 描述子线的长度  
                    end_x = int(x + length * np.cos(angle))  
                    end_y = int(y + length * np.sin(angle))  
                    cv2.line(image_raw, (x, y), (end_x, end_y), color=(255, 0, 0), thickness=2)  # 绘制描述子的方向线  
            
            """
            cv2.imwrite(f"./tmp/output_{filename}", out)


def load_model():
    rknn = RKNNLite()

    print("-->loading model")
    rknn.load_rknn("./super_point.rknn")
    print("loading model done")

    print("--> Init runtime environment")
    ret = rknn.init_runtime()
    if ret != 0:
        print("Init runtime environment failed")
        exit(ret)

    print("done")
    return rknn


if __name__ == "__main__":
    input_folder = "icl_snippet"
    images_array = load_images_from_folder(input_folder)
