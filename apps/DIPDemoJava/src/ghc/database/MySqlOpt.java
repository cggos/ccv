package ghc.database;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;

import com.mysql.jdbc.Statement;

public class MySqlOpt {

    // MySQL��JDBC URL��д��ʽ��jdbc:mysql://�������ƣ����Ӷ˿�/���ݿ������?����=ֵ
    // ������������Ҫָ��useUnicode��characterEncoding
	
	private static final String JDBC_DRIVER = "com.mysql.jdbc.Driver";
	private static final String HOST = "jdbc:mysql://localhost:3306/";
	private static final String DATABASE = "iccarddb";
	private static final String USER = "root";
	private static final String PASSWORD = "";
	
	private Connection conn;
	public Statement statement;
	
	public MySqlOpt(){   	
        try {
        	// ��̬����mysql����
			Class.forName(JDBC_DRIVER);
	        // or:
	        // com.mysql.jdbc.Driver driver = new com.mysql.jdbc.Driver();
	        // or��
	        // new com.mysql.jdbc.Driver();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        System.out.println("�ɹ�����MySQL��������");
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
	            System.out.println("�������ݱ�ɹ�");
	            
	            sql = "insert into student1(NO,name) values('2012001','��ΰ��')";
	            result = statement.executeUpdate(sql);

	            sql = "select * from student1";
	            ResultSet rs = statement.executeQuery(sql);//executeQuery�᷵�ؽ���ļ��ϣ����򷵻ؿ�ֵ
	            System.out.println("ѧ��\t����");
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