package LibPurple.utils;


import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * A classic logger class.
 * $loggingLevel: Minimum level for prints.
 * The included functions will print to the DriverStation if you want to print something with
 * level higher that $loggingLevel.
 */
public class Logger
{

	public static enum Level
	{
		DEBUG, INFO, WARNING, ERROR, CRITICAL
	}

	public static Level loggingLevel = Level.DEBUG;
	private static boolean logToFile = false;

	private static String filename = getTimestamp() + ".log";

	public static void setLoggingLevel(Level l)
	{
		loggingLevel = l;
	}

	public static boolean isLogToFile()
	{
		return logToFile;
	}

	public static void setLogToFile(boolean logToFile)
	{
		Logger.logToFile = logToFile;
	}

	public static Level getLoggingLevel()
	{
		return loggingLevel;
	}

	public static void print(String msg, Level lvl)
	{
		if(isPrintNeeded(lvl))
		{
			Utils.print(msg);
			if(logToFile)
			{
				appendToFile(msg, lvl);
			}
		}
	}

	public static void printErr(String msg, Level lvl)
	{
		if(isPrintNeeded(lvl))
		{
			Utils.printErr(msg);
			if(logToFile)
			{
				appendToFile(msg, lvl);
			}
		}
	}

	private static boolean isPrintNeeded(Level lvl)
	{
		return lvl.ordinal() >= loggingLevel.ordinal();
	}

	private static void appendToFile(String s, Level l)
	{
		try(FileWriter fw = new FileWriter(filename, true); // Stackoverflow???
		    BufferedWriter bw = new BufferedWriter(fw);
		    PrintWriter out = new PrintWriter(bw))
		{
		    out.println(l + ": " + s);
		} catch (IOException e) {
		    e.printStackTrace();
		}
	}

	private static String getTimestamp()
	{
		return new SimpleDateFormat("yyyy-MM-dd HH-mm-ss").format(new Date());
	}

}
