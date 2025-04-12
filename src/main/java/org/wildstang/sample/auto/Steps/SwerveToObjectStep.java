package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.swerve.SwerveDrive.DriveType;
import org.wildstang.sample.subsystems.targeting.WsPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;



public class SwerveToObjectStep extends AutoStep {

    private final double startingPower = 0.7;//initial power limit at the start
    private final double timeToMaxSpeed = 0.2;//time until full speed
    private SwerveDrive swerve;
    private double timeout;
    private WsPose wspose;
    private CoralPath coralPath;
    private boolean seenCoral = false;

    private Pose2d fieldAutoPose;

    private Timer timer;

    public SwerveToObjectStep(SwerveDrive drive, Pose2d pose, double newTimeout) {
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        swerve = drive;
        timer = new Timer();
        fieldAutoPose = pose;
        wspose = (WsPose) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_POSE);
        timeout = newTimeout;
    }
    public SwerveToObjectStep(SwerveDrive drive, Pose2d pose){
        this(drive, pose, 0.25);
    }

    @Override
    public void initialize() {
        swerve.setToAuto();
        timer.start();
    }

    @Override
    public void update() {
        if (timer.hasElapsed(timeout) && wspose.coralInView()) seenCoral = true;

        if (seenCoral){
            swerve.setDriveState(DriveType.CORALINTAKE);
            if (swerve.isAtPosition() || coralPath.hasCoral()) {
                swerve.setDriveState(DriveType.AUTO);
                setFinished();
            }
        } else {
            swerve.setAutoValues(0,0,0.0,0.0, fieldAutoPose);
            if (swerve.isAtPosition() || coralPath.hasCoral()){
                setFinished();
            }
        }

        swerve.setAutoScalar(Math.min(startingPower + timer.get() * (1 - startingPower)/(timeToMaxSpeed), 2.0));
    }

    @Override
    public String toString() {
        return "Swerve To Object Step";
    }
}
