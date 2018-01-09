package LibPurple.watchdog;

import java.util.ArrayList;
import java.util.List;

public class WatchdogLib {
	private static List<Watchdog> watchdogs = new ArrayList<>();
	private static Thread wdThread;
	private static WatchdogRunnable wdRunnable;

	public static void addWatchdog(Watchdog wd)
	{
		watchdogs.add(wd);
	}

	public static void triggerWatchdogs()
	{
		for(Watchdog wd : watchdogs)
		{
			wd.execute();
		}
	}

	public static void startWatchdogThread(long millisDelay)
	{
		if(wdThread == null)
		{
			wdRunnable = new WatchdogRunnable(millisDelay);
			wdThread = new Thread(wdRunnable);
		}
		if(!wdThread.isAlive())
			wdThread.start();
	}

	public static void stopWatchdogThread()
	{
		if(wdThread != null && wdThread.isAlive())
		{
			wdRunnable.stop();
		}
	}

}

class WatchdogRunnable implements Runnable
{
	private volatile boolean exit = false;
	private long millisDelay;

	public WatchdogRunnable(long millisDelay) {
		this.millisDelay = millisDelay;
	}

	@Override
	public void run() 
	{
		while(!exit)
		{
			if(Thread.currentThread().isInterrupted())
				break;
			WatchdogLib.triggerWatchdogs();
			try {
				Thread.sleep(millisDelay);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public void stop()
	{
		exit = true;
	}
}

interface Watchdog 
{
	public void execute();
}