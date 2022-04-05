package ghc.dip.opencv;

import ghc.dip.java.DIPJava;
import ghc.files.FileOpt;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;

public class DIPOpenCV extends DIPJava{

	static{ 
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME); 
		//System.loadLibrary("opencv_java249"); 
	}
	
	public static Mat BufferedImage2Mat(BufferedImage image){	
		byte[] data = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
		
		Mat mat = null;
		if(image.getType() == BufferedImage.TYPE_BYTE_GRAY){
			mat = new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC1);
		}
		if(image.getType() == BufferedImage.TYPE_3BYTE_BGR){
			mat = new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC3);
		}      
        mat.put(0, 0, data);
            
        return mat;
	}
	
	public static BufferedImage Mat2BufferedImage(Mat mat){
		byte[] data = new byte[mat.rows() * mat.cols() * (int)(mat.elemSize())];
		mat.get(0, 0, data);
		
		BufferedImage image = null;
		if (mat.type() == CvType.CV_8UC1) {
			image = new BufferedImage(mat.cols(),mat.rows(), BufferedImage.TYPE_BYTE_GRAY);
		}
		if (mat.type() == CvType.CV_8UC3) {//”–Œ Ã‚
			image = new BufferedImage(mat.cols(),mat.rows(), BufferedImage.TYPE_3BYTE_BGR);
		}     
        image.getRaster().setDataElements(0, 0, mat.cols(), mat.rows(), data);
        
        return image;
	}
	
	public static Mat read(String pathImg){
		return Highgui.imread(pathImg);
	}
	
	public static void write(String dirImg,String filenameImg,Mat mat){
			
		FileOpt.directoryCheckAndMake(dirImg);
		
		Highgui.imwrite(dirImg+"\\"+filenameImg, mat);
	}
	
	public static Mat RGB2Gray(Mat mat){
		Mat matGray = new Mat(mat.rows(),mat.cols(),CvType.CV_8UC1);
		Imgproc.cvtColor(mat, matGray, Imgproc.COLOR_RGB2GRAY);
		
		return matGray;
	}
}
