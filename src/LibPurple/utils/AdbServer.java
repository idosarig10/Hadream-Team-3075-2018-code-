package LibPurple.utils;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.NoSuchElementException;
import java.util.Scanner;

import org.usfirst.frc.team3075.robot.Robot;

public class AdbServer extends Thread
{ 
	private String messege;
	public static ServerSocket ss;
	public Socket conn;
	public Scanner scanner;

	public AdbServer()
	{
		//		thread = new Thread(this);
	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		try {
			connect(3076);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void connect(int port) throws IOException
	{
		Thread t = new Thread(new Runnable() {

			@Override
			public void run()
			{
				// TODO Auto-generated method stub
				try
				{
					Thread.sleep(500);
				} 
				catch (InterruptedException e) 
				{
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				Robot.adb.restartApp();
			}
		});

		Utils.print("1\n");

		if(ss == null)
			ss = new ServerSocket(port);
		else
		{
			ss.close();
			ss = new ServerSocket(port);
		}

		Utils.print("2\n");
		t.start();
		conn = ss.accept();
		Utils.print("Mazal tov\n");
		Scanner s = new Scanner(conn.getInputStream());
		scanner = s;
	}

	public String getString()
	{
		if(ss != null && conn != null && scanner != null)
		{
			try 
			{
				conn.getInputStream().skip(conn.getInputStream().available()); // Flush buffer
			}
			catch (IOException e) 
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			try{
				return scanner.nextLine();
			}
			catch (NoSuchElementException e) {
				// TODO: handle exception
				return null;
			}
		}
		else
			return null;

	}

	public Double getDouble()
	{
		try 
		{
			conn.getInputStream().skip(conn.getInputStream().available()); // Flush buffer
		}
		catch (IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return scanner.nextDouble();
	}

	public Point3075 getCenter()
	{
		Point3075 p = new Point3075();

		String line = getString();

		if(line != null)
		{
			p.x = Double.parseDouble(line.split(" ")[0]);
			p.y = Double.parseDouble(line.split(" ")[1]);
		}
		else
		{
			p.x = -2;
			p.y = -2;
		}

		return p;

	}

	public void close()
	{
		if(ss != null && scanner != null && conn != null)
		{
			try {
				ss.close();
				this.scanner.close();
				this.conn.close();
			}catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
}
