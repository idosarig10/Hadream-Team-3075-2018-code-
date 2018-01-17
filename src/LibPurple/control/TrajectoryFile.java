package LibPurple.control;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.rmi.CORBA.Util;

import LibPurple.utils.Utils;


/* CSV File format
 * time,position,vel,accel
 * 1,5,6,13
 * 2,5,7,14
 */

public class TrajectoryFile extends Trajectory3075 {

	private class TimeStamp
	{
		public double time, position, velocity, acceleration;
		
		public TimeStamp(double time, double position, double velocity, double acceleration)
		{
			this.time = time;
			this.position = position;
			this.velocity = velocity;
			this.acceleration = acceleration;
		}
	}
	
	private List<TimeStamp> stamps;
	
	private double totalTime = 0;
	private double distance = 0;
	private int direction;
	
	public TrajectoryFile(String fileName)
	{
		this(fileName, false);
	}

	public TrajectoryFile(String fileName, boolean isReversed) 
	{
		if(isReversed) this.direction = -1;
		else this.direction = 1;
		
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
			br.readLine(); //Ignore first line 'cause Alon said so
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
	public Setpoint calculate(double time)
	{
		try {
			TimeStamp currStamp = null; //why 2 lines?
			
			if(time >= totalTime)
			{
				currStamp = stamps.get(stamps.size()-1);
			}
			else
			{
				currStamp = stamps.get((int) ((time / totalTime) * stamps.size()));
			}
			
			setpoint.position = currStamp.position * direction;
			setpoint.velocity = currStamp.velocity * direction;
			setpoint.acceleration = currStamp.acceleration * direction;
			
			return setpoint;
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			return null;
		}
	}

	@Override
	public double getTotalTime() 
	{
		return totalTime;
	}

	@Override
	public double getDistance()
	{
		return this.distance * direction;
	}

	@Override
	public void setDistance(double distance)
	{
		this.distance = distance;
	}

	@Override
	public double getDirection() {
		// TODO Auto-generated method stub
		return 0;
	}

}
