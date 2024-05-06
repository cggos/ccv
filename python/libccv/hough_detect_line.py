import numpy as np
import random


def create_img(data, row=240, col=320):
    image = np.zeros((row, col))
    for i in range(len(data)):
        image[int(data[i][1])][int(data[i][0])] = 255
        if abs(data[i][0]) >= col or abs(data[i][1]) >= row:
            continue
        else:
            image[int(data[i][1])][int(data[i][0])] = 255
    return image


def hough_detect_line(img):
    rows, cols = img.shape
    diag_len = np.ceil(np.sqrt(rows**2 + cols**2))

    thetas = np.deg2rad(np.arange(0, 180))
    rhos = np.linspace(-diag_len, diag_len, int(2 * diag_len))

    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)
    num_theta = len(thetas)

    # vote
    vote = np.zeros((int(2 * diag_len), num_theta), dtype=np.uint64)

    # 返回非0位置索引
    y_inx, x_inx = np.nonzero(img)

    # vote in hough space
    for i in range(len(x_inx)):
        x = x_inx[i]
        y = y_inx[i]
        for j in range(num_theta):
            rho = round(x * cos_t[j] + y * sin_t[j]) + diag_len
            if isinstance(rho, int):
                vote[rho, j] += 1
            else:
                vote[int(rho), j] += 1

    return vote, rhos, thetas


def longest_line(vote, rhos, thetas):
    """
    从霍夫矩阵中找到最长的一条线
    """
    # look for peaks
    idx = np.argmax(vote)
    ##下面两句是寻找投票器最大值所对应的行与列，最大值对应的行就是rho的索引，对应的列就是theta的索引
    # 可以用这句代替：row,col=np.unravel_index(idx,ccumulator.shape)
    # rho=rho[row],theta=theta[col]
    rho = rhos[int(idx / vote.shape[1])]
    theta = thetas[idx % vote.shape[1]]
    k = -np.cos(theta) / np.sin(theta)
    b = rho / np.sin(theta)
    return k, b


def main():
    k = 0.5
    b = 1.5
    N = 100
    data = np.zeros([N, 2])
    for i in range(N):
        x = i
        y = k * x + b + random.random()
        data[i][0] = x
        data[i][1] = y

    img = create_img(data)

    vote, rhos, thetas = hough_detect_line(img)

    k, b = longest_line(vote, rhos, thetas)

    print("k = {}, b = {}".format(k, b))


if __name__ == "__main__":
    main()
