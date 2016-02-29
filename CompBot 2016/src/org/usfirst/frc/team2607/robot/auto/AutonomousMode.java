package org.usfirst.frc.team2607.robot.auto;

import java.io.File;
import java.io.FileReader;
import java.util.Scanner;

import org.usfirst.frc.team2607.robot.Robot;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.io.TextFileDeserializer;

abstract public class AutonomousMode {

	public Robot robot;
	
	AutonomousMode(Robot r){
		robot = r;
	}
	
	AutonomousMode(){
		
	}
	
	public abstract void run();
	
	public abstract String getName();
	
	protected Path getPathFromFile(String filename){
    	Path path = null;
		try {
			Scanner trajFile = new Scanner(new FileReader(new File(filename)));
			trajFile.useDelimiter("\\Z");
			String traj = trajFile.next();
			TextFileDeserializer tfds = new TextFileDeserializer();
			path = tfds.deserialize(traj);
		} catch (Exception e) {
			System.err.println("Path retreival from file failed");
			e.printStackTrace();
		}
		return path;
	}
}