package ghc.windows;

import ghc.files.FileOpt;

import java.awt.Container;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JTextField;

public class FileDownloadDlg extends JDialog {

	private static final long serialVersionUID = 1L;

	public FileDownloadDlg(Frame owner) {
		super(owner,"文件下载",true);
		// TODO Auto-generated constructor stub		
		setBounds(200, 200, 500, 200);
		setLayout(null);
		
		Container container = getContentPane();
			
		JLabel labelURL = new JLabel("文件URL：");
		labelURL.setBounds(10, 10, 80, 30);
		JLabel labelFileName = new JLabel("文件名：");
		labelFileName.setBounds(10, 50, 80, 30);
		
		JTextField txtURL = new JTextField();
		txtURL.setBounds(100, 10, 350, 30);
		txtURL.setText("http://");
		txtURL.setFocusable(true);
		JTextField txtFileName = new JTextField();
		txtFileName.setBounds(100, 50, 350, 30);
		
		JButton btnDownload = new JButton("文件下载");
		btnDownload.setBounds(180, 100, 120, 40);
		btnDownload.addActionListener(new ActionListener() {	
			@Override
			public void actionPerformed(ActionEvent arg0) {
				// TODO Auto-generated method stub		
				if(txtURL.getText().equals("")){
					MessageBox.waringShow("请输入文件URL！");
					return;
				}
				if(txtFileName.getText().length() == 0){
					MessageBox.waringShow("请输入文件名！");
					return;
				}
				FileOpt.downloadFile(txtURL.getText(),txtFileName.getText());			
				MessageBox.waringInfo("文件下载成功！");
			}
		});
		
		container.add(labelURL);
		container.add(txtURL);
		container.add(labelFileName);
		container.add(txtFileName);
		container.add(btnDownload);
	}
}
