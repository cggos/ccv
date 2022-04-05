# DIPDemoJava
Digital Image Processing Demonstration with OpenCV 2 and Java.

---

## Development Requirements
* System: Windows 7 (32bit)
* JDK ==> jdk1.8.0_51
* Eclipse ==> eclipse-java-mars-R-win32
* EGit ==> http://download.eclipse.org/egit/updates/
* OpenCV ==> OpenCV 2.4.9 for Windows
* MySQL Connector/J ( the official JDBC driver for MySQL ) ==> mysql-connector-java-5.1.39

## Eclipse
### Shortcuts
* Shift + Alt + S: Override/Implement Method...
* Shift + Alt + R: Rename files

### Intellisense
[http://blog.csdn.net/ysydao/article/details/38731069](http://blog.csdn.net/ysydao/article/details/38731069 "eclipse智能提示及快捷键")

## Image Processing
### OpenCV
* Using OpenCV Java with Eclipse: [http://docs.opencv.org/2.4/doc/tutorials/introduction/java_eclipse/java_eclipse.html#java-eclipse](http://docs.opencv.org/2.4/doc/tutorials/introduction/java_eclipse/java_eclipse.html#java-eclipse "Using OpenCV Java with Eclipse")
* OpenCV Java Tutorials documentation: [http://opencv-java-tutorials.readthedocs.io/en/latest/](http://opencv-java-tutorials.readthedocs.io/en/latest/ "OpenCV Java Tutorials documentation")
### Java
* Java Digital Image Processing Tutorial: [http://www.tutorialspoint.com/java_dip/](http://www.tutorialspoint.com/java_dip/ "Java Digital Image Processing Tutorial")
* Graphics

## GUI Programming
[https://www.ntu.edu.sg/home/ehchua/programming/java/J4a_GUI.html](https://www.ntu.edu.sg/home/ehchua/programming/java/J4a_GUI.html "Java Programming Tutorial ---- Programming Graphical User Interface (GUI)")

* JStatusBar: [http://java-articles.info/articles/?p=65](http://java-articles.info/articles/?p=65)

## MySQL Programming with Java
* MySQL Connector/J: [http://dev.mysql.com/downloads/connector/j/](http://dev.mysql.com/downloads/connector/j/)
* JDBC Tutorial: [http://www.tutorialspoint.com/jdbc/index.htm](http://www.tutorialspoint.com/jdbc/index.htm)
* MySQL Java tutorial: [http://zetcode.com/db/mysqljava/](http://zetcode.com/db/mysqljava/)

## Excel Programming with Java
* Java Excel API: [http://jexcelapi.sourceforge.net/](http://jexcelapi.sourceforge.net/ "Java Excel API - A Java API to read, write, and modify Excel spreadsheets")
* JAVA操作Excel文件:[http://www.cnblogs.com/wuxinrui/archive/2011/03/20/1989326.html](http://www.cnblogs.com/wuxinrui/archive/2011/03/20/1989326.html)

## Java Regular Expressions
* [http://www.runoob.com/java/java-regular-expressions.html](http://www.runoob.com/java/java-regular-expressions.html "Java 正则表达式")

## Events Processing
* Key Bindings: [http://docs.oracle.com/javase/tutorial/uiswing/misc/keybinding.html](http://docs.oracle.com/javase/tutorial/uiswing/misc/keybinding.html "How to Use Key Bindings")

### Issues
Close frame by pressing the escape key:

    //once ESC is pressed to quit the program
    AbstractAction actionExit = new AbstractAction(){ 
    	private static final long serialVersionUID = 1L;
    	public void actionPerformed(ActionEvent e){
    		dispose();
    		//System.exit(0);
    	}
    };
    getRootPane().getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW).put(
    		KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0), "EXIT");
    getRootPane().getActionMap().put("EXIT", actionExit);

## Multi-Threading

## Package to Jar file
* 把Java程序打包成jar文件包并执行: [http://www.cnblogs.com/mq0036/p/3885407.html](http://www.cnblogs.com/mq0036/p/3885407.html)
* MANIFEST.MF

## 《Java从入门到精通》（明日科技）
![《Java从入门到精通》京东产品特色](http://i.imgur.com/5QTa0Xh.jpg)