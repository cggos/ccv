#include <pangolin/pangolin.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {
    pangolin::CreateWindowAndBind("Multi Image", 752, 480);

    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(752, 480, 420, 420, 320, 320, 0.1, 1000),
        pangolin::ModelViewLookAt(-2, 0, -2, 0, 0, 0, pangolin::AxisY));

    // 创建三个视图，后两个视图并不是交互视图，因此我们没有使用setHandler()设置视图句柄
    pangolin::View& d_cam = pangolin::Display("cam")
                                .SetBounds(0., 1., 0., 1., -752 / 480.)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& cv_img_1 = pangolin::Display("image_1")
                                   .SetBounds(2 / 3.0f, 1.0f, 0., 1 / 3.0f, -752 / 480.)
                                   .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::View& cv_img_2 = pangolin::Display("image_2")
                                   .SetBounds(0., 1 / 3.0f, 2 / 3.0f, 1.0, 752 / 480.)
                                   .SetLock(pangolin::LockRight, pangolin::LockBottom);

    // 创建glTexture容器用于读取图像
    // 图像的宽度和高度一定要设置为和原图片一致
    // 由于我们使用OpenCV从文件中读取并存储图像，cv::Mat的图像存储顺序为BGR，而数据存储格式为uint型，
    // 因此最后两个参数我们分别设置为GL_BGR和GL_UNSIGNED_BYTE，
    // 至于pangolin的内部存储格式，对图片的显示影响不大，因此一般都设置为GL_RGB
    pangolin::GlTexture imgTexture1(752, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    pangolin::GlTexture imgTexture2(752, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glColor3f(1.0f, 1.0f, 1.0f);
        pangolin::glDrawColouredCube();

        // 从文件读取图像，向GPU装载图像
        cv::Mat img1 = cv::imread("../data/mh01_01.png");
        cv::Mat img2 = cv::imread("../data/mh01_02.png");
        imgTexture1.Upload(img1.data, GL_BGR, GL_UNSIGNED_BYTE);
        imgTexture2.Upload(img2.data, GL_BGR, GL_UNSIGNED_BYTE);

        // 显示图像
        cv_img_1.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);          // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture1.RenderToViewportFlipY();  // 需要反转Y轴，否则输出是倒着的

        cv_img_2.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);
        imgTexture2.RenderToViewportFlipY();

        pangolin::FinishFrame();
    }

    return 0;
}
