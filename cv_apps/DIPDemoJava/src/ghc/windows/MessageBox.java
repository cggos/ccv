package ghc.windows;

import javax.swing.JOptionPane;

public class MessageBox {

	public static void waringShow(String message){
		Object[] options = { "OK" }; 
		JOptionPane.showOptionDialog(
				null, 
				message, 
				"����", 
				JOptionPane.DEFAULT_OPTION, 
				JOptionPane.WARNING_MESSAGE, 
				null, 
				options, 
				options[0]);
	}
	public static void waringInfo(String message){		
//		JOptionPane.showInternalConfirmDialog(null, 
//				"please choose one", "information", 
//				JOptionPane.YES_NO_CANCEL_OPTION, 
//				JOptionPane.INFORMATION_MESSAGE); 
		
//		JOptionPane.showConfirmDialog(
//				null, 
//				"choose one", 
//				"choose one", 
//				JOptionPane.YES_NO_OPTION); 
		
		JOptionPane.showMessageDialog(
				null, 
				message, 
				"��Ϣ��ʾ", 
				JOptionPane.INFORMATION_MESSAGE); 

	}
}
