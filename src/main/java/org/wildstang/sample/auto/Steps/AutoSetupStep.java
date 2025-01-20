package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.LED.LedController;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.swerve.SwerveDrive.driveType;
import org.wildstang.sample.subsystems.targeting.WsPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoSetupStep extends AutoStep{

    private double x, y, heading;
    private SwerveDrive swerve;
    private WsPose pose;

    /**
     * Setup the robot at start of autonomous
     * @param x // Choreo blue alliance starting x position (m)
     * @param y // Choreo blue allinace starting y position (m)
     * @param pathHeading CW Heading
     * @param alliance // Set alliance in Core
     */
    public AutoSetupStep(double x, double y, double pathHeading, Alliance alliance){
        this.x = x;
        this.y = y;
        heading = pathHeading;
        Core.setAlliance(alliance);
        LedController led = (LedController) Core.getSubsystemManager().getSubsystem(WsSubsystems.LED);
    }

    public void update(){
        double difference = Math.min(Math.abs(swerve.getGyroAngle() - heading), Math.abs(360 - (swerve.getGyroAngle() - heading)));
        // Gyro reset and reads within 1 degree of what we told it to
        if (difference < 1) {
            // Sets odometry alliance relative
            pose.resetPose(new Pose2d(x,y, Rotation2d.fromDegrees(360 - heading)));
            this.setFinished();
        }
    }
    public void initialize(){
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        pose = (WsPose) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_POSE);
        swerve.setAutoValues(0, 0, 0, 0);
        swerve.setAutoHeading(heading);
        swerve.setGyro(heading);
    }
    public String toString(){
        return "Auto Setup Step";
    }
    
}
