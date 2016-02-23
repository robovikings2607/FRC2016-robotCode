package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.PrintWriter;
import java.lang.reflect.Array;
import java.util.Scanner;
import java.util.Vector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousEngine implements Runnable {
	Timer autoTimer;
	private RobotDrive rdrive;
	private PuncherArm arm;
	private Solenoid shifter;

	int step;
	int mode;


	public AutonomousEngine(RobotDrive drive, PuncherArm arm, Solenoid shifter){
		rdrive = drive;
		this.arm = arm;
		this.shifter = shifter;
		autoTimer = new Timer();
		step = 0;
		mode = 0;
	}

	public void displayMode() {
		SmartDashboard.putNumber("autoMode", mode);
		switch(mode) {
			case 0:
				SmartDashboard.putString("autonMode", "Auton: NONE");
				break;
			
			default:
				SmartDashboard.putString("autonMode", "UNKNOWN!!");
				break;
		}	
	
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
		if (++mode > 16) mode = 0;
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
			switch (mode) {
			case 0:
				// turn off outputs
				break;
			case 1:
				System.out.println("Running Auto 1");
//				doSomethingInteresting();
				mode = 0;
				break;

				default:
			}
		System.out.println("Exiting Auto");

	}



}