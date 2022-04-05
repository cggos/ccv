package ghc.files;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class LoggerUtil {
	
    private static String getLogPath() {  
    	String dirLog = "DIPDemoJava程序日志";
        StringBuffer logPath = new StringBuffer();  
        logPath.append(System.getProperty("user.home"));  
        logPath.append("\\"+dirLog);  
        File file = new File(logPath.toString());  
        if (!file.exists())  
            file.mkdir();  
          
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd");  
        logPath.append("\\"+sdf.format(new Date())+".log");  
          
        return logPath.toString();  
    }  
      
    public static void setLogingProperties(Logger logger) throws SecurityException, IOException {  
        setLogingProperties(logger,Level.ALL);  
    }  
      
    public static void setLogingProperties(Logger logger,Level level) {  
        FileHandler fh;  
        try {  
            fh = new FileHandler(getLogPath(),true);  
            logger.addHandler(fh);//日志输出文件  
            //logger.setLevel(level);  
            fh.setFormatter(new SimpleFormatter());//输出格式  
            //logger.addHandler(new ConsoleHandler());//输出到控制台  
        } catch (SecurityException e) {  
            logger.log(Level.SEVERE, "安全性错误", e);  
        } catch (IOException e) {  
            logger.log(Level.SEVERE,"读取文件日志错误", e);  
        }  
    } 
    
    //有问题
    public static void writeInfoLog(String strInfo){
		Logger logger = Logger.getLogger("sgg");
        try {  
            LoggerUtil.setLogingProperties(logger);  
            logger.log(Level.INFO, strInfo);  
        } catch (SecurityException e) {  
            // TODO Auto-generated catch block  
            e.printStackTrace();  
        } catch (IOException e) {  
            // TODO Auto-generated catch block  
            e.printStackTrace();  
        }  
    }
}
