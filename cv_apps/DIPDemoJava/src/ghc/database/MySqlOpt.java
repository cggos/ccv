package ghc.database;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;

import com.mysql.jdbc.Statement;

public class MySqlOpt {

    // MySQL的JDBC URL编写方式：jdbc:mysql://主机名称：连接端口/数据库的名称?参数=值
    // 避免中文乱码要指定useUnicode和characterEncoding
	
	private static final String JDBC_DRIVER = "com.mysql.jdbc.Driver";
	private static final String HOST = "jdbc:mysql://localhost:3306/";
	private static final String DATABASE = "iccarddb";
	private static final String USER = "root";
	private static final String PASSWORD = "";
	
	private Connection conn;
	public Statement statement;
	
	public MySqlOpt(){   	
        try {
        	// 动态加载mysql驱动
			Class.forName(JDBC_DRIVER);
	        // or:
	        // com.mysql.jdbc.Driver driver = new com.mysql.jdbc.Driver();
	        // or：
	        // new com.mysql.jdbc.Driver();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        System.out.println("成功加载MySQL驱动程序");
	}
	
	public void connectDB(){
        try {
//            String url = "jdbc:mysql://localhost:3306/iccarddb?"
//                    + "user=root&password=&useUnicode=true&characterEncoding=UTF8";
//			conn = DriverManager.getConnection(url);
        	conn = DriverManager.getConnection(HOST+DATABASE,USER,PASSWORD);
			statement = (Statement) conn.createStatement();
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}      
	}

	public void createTable(){
		String sql = "create table student1(NO char(20),name varchar(20),primary key(NO))";
		int result;
		try {
			result = statement.executeUpdate(sql);
			if (result != -1) {
	            System.out.println("创建数据表成功");
	            
	            sql = "insert into student1(NO,name) values('2012001','陶伟基')";
	            result = statement.executeUpdate(sql);

	            sql = "select * from student1";
	            ResultSet rs = statement.executeQuery(sql);//executeQuery会返回结果的集合，否则返回空值
	            System.out.println("学号\t姓名");
	            while (rs.next()) {
	                System.out.println(rs.getString(1) + "\t" + rs.getString(2));
	            }
			}
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void close(){
		try {
			conn.close();
			statement.close();
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}