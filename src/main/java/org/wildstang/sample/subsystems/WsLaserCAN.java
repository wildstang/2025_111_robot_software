package org.wildstang.sample.subsystems;

import java.util.Arrays;

import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsLaserCAN {

    private double thresholdDistance;

    // Array used to store the last 9 laserCAN measurements
    private double[] savedLcDistance = { Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, 
        Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE };
    private LaserCan lc;
    
    public WsLaserCAN(int id, double distance) {
        lc = new LaserCan(id);
        thresholdDistance = distance;
    }

    // Update with latest laserCAN measurements
    public void updateMeasurements() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            //System.out.println(measurement.distance_mm);
            double tempvalue = savedLcDistance[0];
            double changeValue = measurement.distance_mm;
            for(int n = 0; n < 9; n++) {
                tempvalue = savedLcDistance[n];
                savedLcDistance[n] = changeValue;
                changeValue = tempvalue;
            }
        } else {
            double tempvalue = savedLcDistance[0];
            double changeValue = 400;
            for(int n = 0; n < 9; n++) {
                tempvalue = savedLcDistance[n];
                savedLcDistance[n] = changeValue;
                changeValue = tempvalue;
            }
        }
    }

    public boolean blocked (double threshold) {
        // LaserCan.Measurement measurement = lc.getMeasurement();
        // if (measurement == null) return false;
        // return threshold > measurement.distance_mm;
        
       int numberCorrect = 0;
        for(int n = 0; n < 9; n++) {
            if(savedLcDistance[n] < threshold) {
                numberCorrect++;
            }
        }
        return numberCorrect > 4;
    }
    public boolean blocked(){
        return blocked(thresholdDistance);
    }
    public void putData(){
        if (lc.getMeasurement() != null){
            SmartDashboard.putNumber("@ Algae LASERCAN", lc.getMeasurement().distance_mm);
            SmartDashboard.putNumber("lasercan saved 1", savedLcDistance[0]);
            SmartDashboard.putNumber("lasercan saved 2", savedLcDistance[1]);
            SmartDashboard.putNumber("lasercan saved 3", savedLcDistance[2]);
            SmartDashboard.putNumber("lasercan saved 4", savedLcDistance[3]);
            SmartDashboard.putNumber("lasercan saved 5", savedLcDistance[4]);
            SmartDashboard.putBoolean("lasercan blocked", blocked());
        }
        
    }
}