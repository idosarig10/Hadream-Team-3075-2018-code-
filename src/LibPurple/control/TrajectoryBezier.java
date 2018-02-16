package LibPurple.control;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import LibPurple.control.Trajectory3075.Setpoint;
import LibPurple.utils.Utils;

public class TrajectoryBezier extends Trajectory3075
{

	private List<TimeStamp> stamps;

	private int lastSavedStampIndex;

	private double totalTime = 0;
	private double distance = 0;

	public TrajectoryBezier(String fileName) 
	{

		stamps = new ArrayList<TimeStamp>();
		setpoint = new Setpoint();

		BufferedReader br = null;

		try {
			br = new BufferedReader(new FileReader(fileName));
		} catch (FileNotFoundException e1) {
			//TODO log
			e1.printStackTrace();	        
			Utils.printErr(e1.getMessage());
			return;
		}

		try
		{
			br.readLine();//ignore first line
			String line = br.readLine();
			String lastLine = "0,";

			while (line != null) 
			{
				addLine(line);
				lastLine = line;
				line = br.readLine();
			}

			totalTime = Double.parseDouble(lastLine.split(",")[0]);
			distance = Double.parseDouble(lastLine.split(",")[1]);

		} catch(Exception e) {
			try {
				br.close();
			} catch (IOException e1) {
				Utils.printErr("[Trajectory File]: Error closing file");
			}
		}

		try {
			br.close();
		} catch (IOException e) {
			Utils.printErr("[Trajectory File]: Error closing file");
		}
	}

	private void addLine(String line) 
	{
		String[] attr = line.split(",");

		double time = Double.parseDouble(attr[0]);
		double pos = Double.parseDouble(attr[1]);
		double vel = Double.parseDouble(attr[2]);
		double accel = Double.parseDouble(attr[3]);

		stamps.add(new TimeStamp(time, pos, vel, accel));
	}

	@Override
	public Setpoint calculate(double time) {
		// TODO Auto-generated method stub
		int i = 0;
		try
		{
			TimeStamp currStamp = null;
			if(time >= totalTime)
			{
				Utils.printErr("Time > Totaltime");
				currStamp = stamps.get(stamps.size()-1);
			}
			else
			{
				currStamp = stamps.get((int) ((time / totalTime) * stamps.size()));
			}

			setpoint.position = currStamp.position;
			setpoint.velocity = currStamp.velocity;
			setpoint.acceleration = currStamp.acceleration;

			return setpoint;

		}
		catch(Exception e)
		{
			Utils.printErr(e.toString());
			Utils.printErr("i= " + i);
			return null;	
		}
	}

	@Override
	public double getTotalTime() {
		// TODO Auto-generated method stub
		return this.totalTime;
	}

	@Override
	public double getDirection() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getDistance() {
		// TODO Auto-generated method stub
		return this.distance;
	}

	@Override
	public void setDistance(double distance) {
		// TODO Auto-generated method stub

	}

}
