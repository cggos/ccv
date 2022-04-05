package ghc.dip.java;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;

import javax.swing.JPanel;

import ghc.datatype.ImageSize;
import ghc.dip.java.DIPJava;

public class CanvasPanel extends JPanel {
	private static final long serialVersionUID = 1L;

	private Image img;
	private DIPTYPE typeDIP;
	private Graphics2D g2;
	private double angDeg;
	
	enum DIPTYPE{
		Display,
		Rotate,
		Shear
	}
	
	public void drawImage(Image image){
		img = image;
		typeDIP = DIPTYPE.Display;
		repaint();
	}
	
	public void rotateImage(double angdeg){
		angDeg = angdeg;
		typeDIP = DIPTYPE.Rotate;
		repaint();
	}
	
	public void shearImage(){
		typeDIP = DIPTYPE.Shear;
		repaint();
	}
	
	@Override
	public void paint(Graphics g) {
		// TODO Auto-generated method stub
		super.paint(g);
		
		if (img == null) {
			return;
		}
		
		ImageSize<Float> sizeImg = DIPJava.getSuitableImageSizeByPanel(img,this);		
		int newImgW = (int)((float)sizeImg.getWidth());
		int newImgH = (int)((float)sizeImg.getHeight());		
		int panelW = this.getWidth();
		int panelH = this.getHeight();		
		int xImg = (panelW-newImgW)/2;
		int yImg = (panelH-newImgH)/2;
		
		g2 = (Graphics2D)g;
		switch(typeDIP){
		case Display:
			break;
		case Rotate:
			g2.rotate(Math.toRadians(angDeg));
			break;
		case Shear:
			g2.shear(0.3, 0);
			break;
		default:
			break;
		}		
		g2.drawImage(img,xImg,yImg,newImgW,newImgH,this);
	}	
}
