package org.wildstang.sample.auto.Steps;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.Optional;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SwerveToPointStep extends AutoStep {

    private final double startingPower = 0.5;//initial power limit at the start
    private final double timeToMaxSpeed = 0.25;//time until full speed
    private SwerveDrive swerve;

    private Pose2d fieldAutoPose;

    private Timer timer;

    public SwerveToPointStep(SwerveDrive drive, Pose2d pose) {
        swerve = drive;
        timer = new Timer();
        fieldAutoPose = pose;
    }

    @Override
    public void initialize() {
        swerve.setToAuto();
        timer.start();
    }

    @Override
    public void update() {
        swerve.setAutoValues(0.0,0.0, fieldAutoPose);
        swerve.setAutoScalar(startingPower + timer.get() * (1 - startingPower)/(timeToMaxSpeed));
        if (swerve.isAtPosition()) {
            swerve.setAutoScalar(2.0);
            setFinished();
        } 
    }

    @Override
    public String toString() {
        return "Swerve To Point Step";
    }
}
