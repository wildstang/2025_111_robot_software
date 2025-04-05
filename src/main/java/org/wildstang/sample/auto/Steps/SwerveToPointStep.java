package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;



public class SwerveToPointStep extends AutoStep {

    private final double startingPower = 0.7;//initial power limit at the start
    private final double timeToMaxSpeed = 0.2;//time until full speed
    private SwerveDrive swerve;
    private double turnStartTime; // Time to start turning to the end heading

    private Pose2d fieldAutoPose;

    private Timer timer;

    public SwerveToPointStep(SwerveDrive drive, Pose2d pose, double turnStart) {
        this.turnStartTime = turnStart;
        swerve = drive;
        timer = new Timer();
        fieldAutoPose = pose;
    }

    public SwerveToPointStep(SwerveDrive drive, Pose2d pose) {
        turnStartTime = 0;
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
        if (timer.hasElapsed(turnStartTime)) {
            swerve.setAutoValues(0,0,0.0,0.0, fieldAutoPose);
        } else {
            swerve.setAutoValues(0,0,0,0,new Pose2d(fieldAutoPose.getTranslation(), swerve.odoAngle()));
        }
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
