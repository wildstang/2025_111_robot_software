package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.swerve.WsSwerveHelper;
import org.wildstang.sample.subsystems.targeting.WsPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;



public class ObjectIntakeStep extends AutoStep {

    private final double startingPower = 0.5;//initial power limit at the start
    private final double timeToMaxSpeed = 0.25;//time until full speed
    private SwerveDrive swerve;
    private WsPose pose;

    private Pose2d fieldAutoPose;

    private Timer timer;

    public ObjectIntakeStep() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        pose = (WsPose) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_POSE);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        swerve.setToAuto();
        timer.start();
    }

    @Override
    public void update() {

        //if (WsSwerveHelper.angleDist(swerve.getGyroAngle(), pose.turnToTarget(null)))

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