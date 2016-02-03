package org.usfirst.frc.team2607.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

/**
 *
 * @author rossron
 */
public class SmoothedEncoder extends Encoder implements Runnable {
    private int numSamples = 15;
    private double[] sampleValues = new double[numSamples];
    private volatile double currentRate;
    int chA;
    public SmoothedEncoder(int chA, int chB, boolean reversed, CounterBase.EncodingType type) {
        super(chA, chB, reversed, type);
        this.chA = chA;
        new Thread(this).start();
    }
            
    public double getCurrentRate() {
        return currentRate;
    }
    
    public double pidGet() {
        return currentRate;
    }

    public void run() {
        while (true) {
        for (int i = sampleValues.length - 1; i > 0; i--) {
            sampleValues[i] = sampleValues[i-1];
        }
        sampleValues[0] = getRate();
        double[] median = Arrays.copyOfRange(sampleValues, 0, numSamples);
        Arrays.sort(median);
        currentRate = median[median.length/2];       
        try { Thread.sleep(5); } catch (Exception e) {}
        }
    }   
   
}