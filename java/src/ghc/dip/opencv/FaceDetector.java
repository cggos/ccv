package ghc.dip.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.highgui.Highgui;
import org.opencv.objdetect.CascadeClassifier;

public class FaceDetector {
	
	static{ 
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME); 
		//System.loadLibrary("opencv_java249"); 
	}
	
	public int nCountFaceDetected;
	
	public FaceDetector(){
		nCountFaceDetected = 0;
	}
	
	public Mat detect(String pathFaceImg){
        CascadeClassifier faceDetector = 
           		new CascadeClassifier(".\\config\\haarcascade_frontalface_alt.xml");

        //FaceDetector.class.getResource(pathFaceImg).getPath()
        Mat image = Highgui.imread(pathFaceImg);

        MatOfRect faceDetections = new MatOfRect();
        faceDetector.detectMultiScale(image, faceDetections);

        nCountFaceDetected = faceDetections.toArray().length;

        for (Rect rect : faceDetections.toArray()) {
            Core.rectangle(
            		image, 
            		new Point(rect.x, rect.y), 
            		new Point(rect.x + rect.width, rect.y + rect.height),
                    new Scalar(0, 255, 0));
        }
        
        return image;
	}
}
