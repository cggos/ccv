#include <string>
#include <fftw3.h>

#include "../CImg.h"
using namespace cimg_library;

int main()
{
	std::string strImgIn  = "/home/cg/projects/cgocv_app/ros_wrapper/src/cgocv/images/lena.bmp";
	std::string strImgOut = "lenaFFTW.bmp";

	CImg<float> imgIn(strImgIn.c_str());
	int N,Nx,Ny;
	N=imgIn.size();
	Nx=imgIn.width();
	Ny=imgIn.height();
	CImg<float> imgOut(Nx,Ny,1,3,0);

	fftwf_complex *in  = (fftwf_complex*)fftwf_malloc(Ny*Nx*sizeof(fftwf_complex));
	fftwf_complex *out = (fftwf_complex*)fftwf_malloc(Ny*Nx*sizeof(fftwf_complex));

	fftwf_plan planDFT2D_F = fftwf_plan_dft_2d(Nx,Ny, in, out,FFTW_FORWARD, FFTW_ESTIMATE);

	int v=0;
	while(v<3)
	{
		for(int x=0;x<Nx;x++)
		{
			for(int y=0;y<Ny;y++)
			{
				in[y+x*Ny][0]=imgIn(x,y,0,v);
			}
		}

		fftwf_execute(planDFT2D_F);

		for(int x=0;x<Nx;x++)
		{
			for(int y=0;y<Ny;y++)
			{
				*(imgOut.data(x,y,v))=sqrt(pow(out[y+x*Ny][0],2)+pow(out[y+x*Ny][1],2));
				if((*(imgOut.data(x,y,v)))>25000)
				{
					*(imgOut.data(x,y,v))=25000;
				}
			}
		}
		v++;
	}

	fftwf_destroy_plan(planDFT2D_F);
	fftwf_cleanup();
	if (in != NULL)
	{
		fftwf_free(in);
	}
	if (out != NULL)
	{
		fftwf_free(out);
	}

	CImg<float> imageOut(imgOut.data(),Nx,Ny,1,3);
	imageOut.normalize(0,255);
	imageOut.save_bmp(strImgOut.c_str());

    return 0;
}
