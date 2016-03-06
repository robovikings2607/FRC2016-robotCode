package org.usfirst.frc.team2607.robot.auto;

import java.io.File;
import java.io.FileInputStream;
import java.io.PrintWriter;
import java.lang.reflect.Array;
import java.util.Scanner;
import java.util.Vector;

import org.usfirst.frc.team2607.robot.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousEngine implements Runnable {
	Timer autoTimer;
//	private RobotDrive rdrive;
//	private PuncherArm arm;
//	private Solenoid shifter;
	private Robot robot;
	private AutonomousManager kidiger;

	int step;
	int mode;


	public AutonomousEngine(Robot robot){
		this.robot = robot;
		autoTimer = new Timer();
		step = 0;
		mode = 1;
		
		kidiger = new AutonomousManager(this.robot);
	}

	public void displayMode() {
		SmartDashboard.putNumber("autoMode", mode);
/*		switch(mode) {
			case 0:
				SmartDashboard.putString("autonMode", "Auton: NONE");
				break;
			
			default:
				SmartDashboard.putString("autonMode", "UNKNOWN!!");
				break;
		}	*/
		SmartDashboard.putString("autonMode", kidiger.getModeByIndex(mode).getName());
	
	}
	
	public int getMode() {
		return mode;
	}
	
	public void saveMode() {
		try {
			PrintWriter p = new PrintWriter(new File("/home/lvuser/autoMode.txt"));
			p.printf("%d", mode);
			p.flush();
			p.close();
		} catch (Exception e) {
			
		}
	}
	
	public void selectMode() {
		if (++mode > (kidiger.modes.size() - 1)) mode = 1;
		saveMode();
		displayMode();
	}
	
	public void setMode(int i) {
		mode = i;
		saveMode();
		displayMode();
	}

	public void loadSavedMode() {
		try {
			FileInputStream fin = new FileInputStream("/home/lvuser/autoMode.txt");
			Scanner s = new Scanner(fin);
			if (s.hasNextInt()) mode = s.nextInt();
			else mode = 0;
			fin.close();
		} catch (Exception e) {
			mode = 0;
		}
		displayMode();
	}
	


	@Override
	public void run() {
		// called by Thread.start();
		System.out.println("Auto Thread Start");
		
		System.out.println("Running Mode: " + kidiger.getModeByIndex(mode).getName());
		kidiger.getModeByIndex(mode).run();
		
		System.out.println("Exiting Auto");

	}
	
	



}