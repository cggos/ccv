#include "cvkit/cv/image.h"
#include "cvkit/maths/mathtools.h"

namespace cg {
    int MatrixRotateImage(int cx, int cy, float degree, float M[3][3]){
        float A[3][3] = {1, 0, -cx,
            0, 1, -cy,
            0, 0,  1};

        float dcos = std::cos(degree);
        float dsin = std::sin(degree);
        float B[3][3] = { dcos, dsin, 0,
            -dsin, dcos, 0,
            0 ,   0 , 1};

        float C[3][3] = {1, 0, cx,
            0, 1, cy,
            0, 0, 1};

        float BA[3][3];
        MatrixMultiply(B, A, BA);
        MatrixMultiply(C, BA, M);
    }

    void ImgProjectiveTransform(const unsigned char  *imgSrc, unsigned int  widthSrc, unsigned int  heightSrc,
                       float H[3][3], unsigned char **imgDst, unsigned int *widthDst, unsigned int *heightDst){
        const int pointsSrc[][2] = { {0,0}, {widthSrc,0}, {0,heightSrc}, {widthSrc,heightSrc} };
        float pointsDst[4][2] = {0};
        for(int i=0; i<4; i++){
            float z = H[2][0]*pointsSrc[i][0] + H[2][1]*pointsSrc[i][1] + H[2][2];
            float zInv = 1.f / z;
            pointsDst[i][0] = (H[0][0]*pointsSrc[i][0] + H[0][1]*pointsSrc[i][1] + H[0][2]) * zInv;
            pointsDst[i][1] = (H[1][0]*pointsSrc[i][0] + H[1][1]*pointsSrc[i][1] + H[1][2]) * zInv;
        }
        float minX = pointsDst[0][0];
        float maxX = pointsDst[0][0];
        float minY = pointsDst[0][1];
        float maxY = pointsDst[0][1];
        for(int i=1; i<4; i++){
            if(pointsDst[i][0] < minX)
                minX = pointsDst[i][0];
            if(pointsDst[i][0] > maxX)
                maxX = pointsDst[i][0];
            if(pointsDst[i][1] < minY)
                minY = pointsDst[i][1];
            if(pointsDst[i][1] > maxY)
                maxY = pointsDst[i][1];
        }
        *widthDst  = static_cast<unsigned int>(std::ceil(maxX - minX));
        *heightDst = static_cast<unsigned int>(std::ceil(maxY - minY));

        unsigned int sizeDst = *widthDst * *heightDst;
        *imgDst = new unsigned char[sizeDst];
        memset(*imgDst, 255, sizeDst);

        float HInv[3][3];
        MatrixInverse3(H, HInv);
        for(int y=(int)minY+1; y<=(int)maxY; ++y){
            for(int x=(int)minX+1; x<=(int)maxX; ++x){
                float xSrc = HInv[0][0]*x + HInv[0][1]*y + HInv[0][2];
                float ySrc = HInv[1][0]*x + HInv[1][1]*y + HInv[1][2];
                float zSrc = HInv[2][0]*x + HInv[2][1]*y + HInv[2][2];
                xSrc /= zSrc;
                ySrc /= zSrc;
                if(xSrc<0 || xSrc>widthSrc || ySrc<0 || ySrc>heightSrc)
                    continue;

                int xS = (int)xSrc;
                int yS = (int)ySrc;

                int xDst = x-(int)minX;
                int yDst = y-(int)minY;
                (*imgDst)[yDst*(*widthDst)+xDst] = imgSrc[yS*widthSrc+xS];
            }
        }
    }

    void ImgProjectiveTransform_Fixed(const unsigned char  *imgSrc, unsigned int widthSrc, unsigned int heightSrc,
                         float H[3][3],     unsigned char **imgDst, unsigned int widthDst, unsigned int heightDst){
        unsigned int sizeDst = widthDst * heightDst;
        *imgDst = new unsigned char[sizeDst];
        memset(*imgDst, 255, sizeDst);

        float HInv[3][3];
        MatrixInverse3(H, HInv);
        for(int y=0; y<heightDst; y++){
            for(int x=0; x<widthDst; x++){

                float xfSrc = HInv[0][0]*x + HInv[0][1]*y + HInv[0][2];
                float yfSrc = HInv[1][0]*x + HInv[1][1]*y + HInv[1][2];
                float zfSrc = HInv[2][0]*x + HInv[2][1]*y + HInv[2][2];
                xfSrc /= zfSrc;
                yfSrc /= zfSrc;

                if(xfSrc<0 || xfSrc>widthSrc || yfSrc<0 || yfSrc>heightSrc)
                    continue;

                int xSrc = (int)xfSrc;
                int ySrc = (int)yfSrc;

                (*imgDst)[y*widthDst+x] = imgSrc[ySrc*widthSrc + xSrc];
            }
        }
    }

}
