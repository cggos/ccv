package ghc.files;

import java.awt.Component;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;

import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;

public class FileOpt {

	public FileOpt() {
		// TODO Auto-generated constructor stub
	}
	
	public static void downloadFile(String urlIn,String fileNameOut){
		try{            
            URL url = new URL(urlIn);
            InputStream inputStream = url.openStream();
            OutputStream outputStream = new FileOutputStream(fileNameOut);
            byte[] buffer = new byte[2048];
            
            int length = 0;
            
            while ((length = inputStream.read(buffer)) != -1) {
               //System.out.println("Buffer Read of length: " + length);
               outputStream.write(buffer, 0, length);
            }
            
            inputStream.close();
            outputStream.close();
            
         }catch(Exception e){
            System.out.println("FileDownloadException: " + e.getMessage());
         }
	}
	
	public static void directoryCheckAndMake(String strDir){
		File dir = new File(strDir);
		if (!dir.exists() && !dir.isDirectory()) {
			dir.mkdir();
		}
	}
	
	public static String chooseFile(Component parent){
		JFileChooser fileChooser = new JFileChooser();
		FileNameExtensionFilter filter = 
				new FileNameExtensionFilter("Í¼ÏñÎÄ¼þ(JPG/PNG/BMP)", "jpg","png","bmp");
		fileChooser.setFileFilter(filter);
		int ret = fileChooser.showOpenDialog(parent);
		String filepath = "";
		if (ret == JFileChooser.APPROVE_OPTION) {
			File selectedFile = fileChooser.getSelectedFile();
			filepath = selectedFile.getPath();
		}
		return filepath;
	}

}
