package ghc.windows;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.GridLayout;
import java.awt.Image;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.KeyStroke;

import org.opencv.core.Mat;

import ghc.dip.java.CanvasPanel;
import ghc.dip.java.DIPJava;
import ghc.dip.opencv.DIPOpenCV;
import ghc.files.FileOpt;
import ghc.threads.MyThread;
import ghc.threads.TimerThread;

public class MainWindow extends JFrame {
	private static final long serialVersionUID = 1L;
	
	private String pathImgSrc;
	private Image imgSrc;
	private Image imgDst;
	
	private TimerThread timerThread;
	private CanvasPanel panelL;
	private CanvasPanel panelR;	
	
	public MainWindow() {
		// TODO Auto-generated constructor stub			
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);	
		setTitle("DIPDemoJava");
        setSize(1024, 640);
        
        setJMenuBar(createMenuBar());		
		JPanel panelClient = new JPanel(new BorderLayout());
		panelClient.add(BorderLayout.CENTER,createMainPanel());
		panelClient.add(BorderLayout.SOUTH, createStatusBar());
		getContentPane().add(panelClient);		
		setVisible(true);
		
		addMainWindowEvents();		

        MyThread mythread = new MyThread();
        Thread t1 = new Thread(mythread);
        Thread t2 = new Thread(mythread);
        t1.start();
        t2.start();      
	}
	
	private JMenuBar createMenuBar(){
		JMenuBar menuBar = new JMenuBar();
		
		JMenu menuFile = new JMenu("文件(F)");
		menuFile.setMnemonic('F');
		JMenuItem menuitemOpen = new JMenuItem("打开(Open)");
		menuitemOpen.setMnemonic('O');
		menuitemOpen.setAccelerator(KeyStroke.getKeyStroke(
				KeyEvent.VK_O,InputEvent.CTRL_MASK));
		JMenuItem menuitemCloseApp = new JMenuItem("退出程序(Exit)");
		menuitemCloseApp.setMnemonic('E');
		menuitemCloseApp.setAccelerator(KeyStroke.getKeyStroke(
				KeyEvent.VK_E,InputEvent.CTRL_MASK|InputEvent.SHIFT_MASK));
		
		JMenu menuDIP = new JMenu("图像处理");
		JMenuItem menuitemGrayImg = new JMenuItem("图像灰度化");
		
		menuBar.add(menuFile);
		menuFile.add(menuitemOpen);
		menuFile.addSeparator();
		menuFile.add(menuitemCloseApp);
		menuBar.add(menuDIP);
		menuDIP.add(menuitemGrayImg);
		
		menuitemOpen.addActionListener(new ActionListener() {			
			@Override
			public void actionPerformed(ActionEvent arg0) {
				// TODO Auto-generated method stub
				pathImgSrc = FileOpt.chooseFile(getContentPane());
				imgSrc = DIPJava.readImage(pathImgSrc);
				panelL.drawImage(imgSrc);
			}
		});
		
		menuitemGrayImg.addActionListener(new ActionListener() {		
			@Override
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
				if(imgSrc == null){
					return ;
				}
				Mat matSrc = DIPOpenCV.read(pathImgSrc);
				
				Mat mat = DIPOpenCV.RGB2Gray(matSrc);
				
				imgDst = DIPOpenCV.Mat2BufferedImage(mat);
				panelR.drawImage(imgDst);
			}
		});
		
		return menuBar;
	}
	
	private JPanel createMainPanel(){
		panelL = new CanvasPanel();
		panelL.setBackground(Color.WHITE);
		panelR = new CanvasPanel();
		panelR.setBackground(Color.WHITE);
		
		JPanel panelMain = new JPanel(new GridLayout(1, 2,5,5));
		panelMain.add(panelL);
		panelMain.add(panelR);
		
		return panelMain;
	}
	
	private JStatusBar createStatusBar(){
		JStatusBar statusBar = new JStatusBar();

		JProgressBar progressBar= new JProgressBar();
		progressBar.setStringPainted(true);	
		statusBar.setLeftComponent(progressBar);

		final JLabel labelDate = new JLabel("dateLabel");
		labelDate.setHorizontalAlignment(JLabel.CENTER);
		statusBar.addRightComponent(labelDate);

		final JLabel labelTime = new JLabel("timeLabel");
		labelTime.setHorizontalAlignment(JLabel.CENTER);
		statusBar.addRightComponent(labelTime);
		
		timerThread = new TimerThread(labelDate,labelTime);
		timerThread.start();
		
		Thread thread = new Thread(new Runnable() {			
			@Override
			public void run() {
				// TODO Auto-generated method stub
				int count = 0;
				while (true) {
					progressBar.setValue(++count);
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					if (count == 100) {
						count = 0;
					}
				}
			}
		});
        thread.start();
		
		return statusBar;
	}
	
	private void addMainWindowEvents(){		    
		addWindowListener(new MainWindowListener());//为窗体添加事件监听器		
		//once ESC is pressed to quit the program
		AbstractAction actionExit = new AbstractAction(){ 
			private static final long serialVersionUID = 1L;
			public void actionPerformed(ActionEvent e){
				timerThread.setRunning(false);
				dispose();
				//System.exit(0);
			}
		};
		getRootPane().getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(
				KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0), "EXIT");
		getRootPane().getActionMap().put("EXIT", actionExit);
	}
}
