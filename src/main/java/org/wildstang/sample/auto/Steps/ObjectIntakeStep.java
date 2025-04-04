package org.wildstang.sample.auto.Steps;

import java.util.Optional;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.swerve.WsSwerveHelper;
import org.wildstang.sample.subsystems.swerve.SwerveDrive.DriveType;
import org.wildstang.sample.subsystems.targeting.VisionConsts;
import org.wildstang.sample.subsystems.targeting.WsPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;



public class ObjectIntakeStep extends AutoStep {

    private final double startingPower = 0.5;//initial power limit at the start
    private final double timeToMaxSpeed = 0.25;//time until full speed
    private SwerveDrive swerve;
    private CoralPath coralPath;

    private Timer timer;

    public ObjectIntakeStep() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        swerve.setToAuto();
        timer.start();
    }

    @Override
    public void update() {

        // Same logic to drive to coral as in teleop
        swerve.setDriveState(DriveType.CORALINTAKE);
        if (swerve.isAtPosition() && coralPath.hasCoral()) {
            swerve.setDriveState(DriveType.AUTO);
            setFinished();
        }
        swerve.setAutoScalar(startingPower + timer.get() * (1 - startingPower)/(timeToMaxSpeed));
    }

    @Override
    public String toString() {
        return "Swerve To Point Step";
    }
}