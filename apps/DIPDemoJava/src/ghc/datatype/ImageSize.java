package ghc.datatype;

public class ImageSize<T extends Number>{
	private T width;
	private T height;
	
	public ImageSize(T w,T h){
		width = w;
		height = h;
	}
	
	public void setSize(T w,T h){
		width = w;
		height = h;
	}
	
	public T getWidth(){
		return width;
	}
	
	public T getHeight(){
		return height;
	}
}
