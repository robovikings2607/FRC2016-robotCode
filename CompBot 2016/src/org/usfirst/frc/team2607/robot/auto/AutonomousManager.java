package org.usfirst.frc.team2607.robot.auto;

import java.util.ArrayList;

import org.usfirst.frc.team2607.robot.Robot;
import org.usfirst.frc.team2607.robot.RobovikingDriveTrainProfileDriver;

import com.team254.lib.trajectory.Path;

/**
 * @author Cerora
 *
 */
public class AutonomousManager {
	
	Robot robot;
	public ArrayList<AutonomousMode> modes = new ArrayList<AutonomousMode>();
	
	AutonomousManager(Robot robot){
		this.robot = robot;
		
		modes.add(new DoNothingFailsafe());
		modes.add(new DoNothing());
		modes.add(new BreachLowBar(robot));
	}
	
	public AutonomousMode getModeByName (String name){
		for (AutonomousMode m : modes){
			if (m.getName().equals(name))
				return m;
		}
		
		try {
			throw new Exception();
		} catch (Exception e) {
			System.err.println("Mode not found");
			e.printStackTrace();
			return new DoNothingFailsafe();
		}
	}
	
	public AutonomousMode getModeByIndex (int index){
		try {
			return modes.get(index);
		} catch (IndexOutOfBoundsException e){
			System.err.println("Mode out of array bounds");
			e.printStackTrace();
			return new DoNothingFailsafe();
		}
	}
	
	
	/*
	 * BEGIN AUTON MODE DECLARATIONS
	 * 
	 * You must add the mode to the array once you define its class
	 */
	

	public class BreachLowBar extends AutonomousMode {

		BreachLowBar(Robot r) {
			super(r);
			// TODO Auto-generated constructor stub
		}

		@Override
		public void run() {
			// TODO Auto-generated method stub
			Path p = getPathFromFile("/home/lvuser/testProfile.txt");
			
			RobovikingDriveTrainProfileDriver mp = new RobovikingDriveTrainProfileDriver(robot.leftMotors,robot.rightMotors, p);
			mp.followPath();
		}

		@Override
		public String getName() {
			return "BreachBarLow";
		}
		
	}
	
	public class DoNothing extends AutonomousMode {
		
		DoNothing(){
			
		}

		@Override
		public void run() {
			System.out.println("Explicitly told not to move");
		}

		@Override
		public String getName() {
			return "DoNothing";
		}
		
	}
	
	public class DoNothingFailsafe extends AutonomousMode {
		
		DoNothingFailsafe(){
			
		}

		@Override
		public void run() {
			System.out.println("This shouldn't be running - Mode 0 selected for some reason");
		}

		@Override
		public String getName() {
			return "DoNothingFailsafe";
		}
		
	}
}
