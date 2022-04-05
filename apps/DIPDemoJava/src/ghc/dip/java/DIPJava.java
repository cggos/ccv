package ghc.dip.java;

import java.awt.Image;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;

import ghc.datatype.ImageSize;

public class DIPJava {
	
	public static Image readImage(String pathImg){
		return Toolkit.getDefaultToolkit().getImage(pathImg);
	}
	
	protected static BufferedImage readBufferedImage(String pathImg){	
		try {
			File input = new File(pathImg);
			BufferedImage bufferedImage = ImageIO.read(input);
			return bufferedImage;
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return null;
		}    
	}
	
	protected static void writeBufferedImage(String pathImg,BufferedImage image,String format){	
        try {
        	File ouptut = new File(pathImg);
			ImageIO.write(image, format, ouptut);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static ImageSize<Float> getSuitableImageSizeByPanel(Image image, JPanel panel)
	{
		Float W_LabelImg = (float) panel.getWidth();
		Float H_LabelImg = (float) panel.getHeight();

		Float W_Img = (float) image.getWidth(null);
		Float H_Img = (float) image.getHeight(null);
		
		ImageSize<Float> newSize = new ImageSize<Float>(W_Img,H_Img);

		//根据显示图像的label大小改变图像尺寸
		if(W_Img>W_LabelImg && H_Img<=H_LabelImg)
		{
			newSize.setSize(W_LabelImg,H_Img/((float)W_Img/W_LabelImg));
		}
		else if(W_Img<=W_LabelImg && H_Img>H_LabelImg)
		{
			newSize.setSize(W_Img/((float)H_Img/H_LabelImg),H_LabelImg);
		}
		else if(W_Img>W_LabelImg && H_Img>H_LabelImg)
		{
			if(W_LabelImg>=H_LabelImg)
			{
				if(W_Img<=H_Img)
				{
					newSize.setSize(W_Img/((float)H_Img/H_LabelImg),H_LabelImg);
				}
				else
				{
					if(W_Img/H_Img >= W_LabelImg/H_LabelImg)
					{
						newSize.setSize(W_LabelImg,H_Img/((float)W_Img/W_LabelImg));
					}
					else
					{
						newSize.setSize(W_Img/((float)H_Img/H_LabelImg),H_LabelImg);
					}
				}
			}
			else
			{
				if(W_Img>=H_Img)
				{
					newSize.setSize(W_LabelImg,H_Img/((float)W_Img/W_LabelImg));
				}
				else
				{
					if(H_Img/(float)W_Img <= H_LabelImg/(float)W_LabelImg)
					{
						newSize.setSize(W_LabelImg,H_Img/((float)W_Img/W_LabelImg));
					}
					else
					{
						newSize.setSize(W_Img/((float)H_Img/H_LabelImg),H_LabelImg);
					}
				}
			}
		}
		return newSize;
	}
}
