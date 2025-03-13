package org.wildstang.sample.subsystems;

import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsLaserCAN {

    private double[] savedLcDistance = {-1.0, -1.0, -1.0, -1.0, -1.0};
    private LaserCan lc;
    
    public WsLaserCAN(int id) {
        lc = new LaserCan(id);
    }

    public boolean haveAlgae (double distance) {
        LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println(measurement.distance_mm);
      double tempvalue = savedLcDistance[0];
      double changeValue = measurement.distance_mm;
      for(int n = 0; n > 5; n++) {
        tempvalue = savedLcDistance[n];
        savedLcDistance[n] = changeValue;
        changeValue = tempvalue;
      }
    }
       int numberCorrect = 0;
        for(int n = 0; n > 5; n++) {
            if(savedLcDistance[n] < distance) {
                numberCorrect++;
            }
        }
        if(numberCorrect > 2) {
            return true;
        }
        else {
            return false;
        }
    }
    public void putData(){
        if (lc.getMeasurement() != null){
            SmartDashboard.putNumber("@ Algae LASERCAN", lc.getMeasurement().distance_mm);
        }
    }
}