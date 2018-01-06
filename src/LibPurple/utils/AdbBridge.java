package LibPurple.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class AdbBridge 
{
	Path binLocation;
	public final static Path DEFAULT_PATH = Paths.get("/usr/bin/adb");
	
	public AdbBridge()
	{
		binLocation = DEFAULT_PATH;
	}
	
	public AdbBridge(Path location)
	{
		binLocation = location;
	}
	
	private boolean runCommand(String args)
	{
		Runtime r = Runtime.getRuntime();
		String cmd = binLocation.toString() + " " + args;
		
		return runShellCommand(cmd);
	}
	
	public boolean runShellCommand(String cmd)
	{
		Runtime r = Runtime.getRuntime();
		
		try{
			Process p = r.exec(cmd);
			p.waitFor();
		}catch (IOException e)
		{
			System.err.println("AdbBridge: could not run command " + cmd);
			e.printStackTrace();
			return false;
		}catch (InterruptedException e)
		{
			System.err.println("AdbBridge: could not run command " + cmd);
			e.printStackTrace();
			return false;
		}
		return true;
	}
	
	public void startAdb()
	{
		runCommand("start-server");
	}
	
	public void removeAllReverseConnections()
	{
		runCommand("reverse --remove-all");
	}
	
	public void stopAdb()
	{
		runCommand("kill-server");
	}
	
	public void restartAdb()
	{
		stopAdb();
		startAdb();
	}
	
	public void reversePortForward(int remotePort, int localPort)
	{
		runCommand("reverse tcp:" + remotePort + " tcp:" + localPort);
	}
	
	public void openApp()
	{
		runCommand("shell am start -n com.example.guy.a3075visionproctest/com.example.guy.a3075visionproctest.MainActivity");
	}
	
	public void closeApp()
	{
		runCommand("shell am force-stop com.example.guy.a3075visionproctest \\;");
	}
	
	public void restartApp()
	{
		closeApp();
		openApp();
	}
}
