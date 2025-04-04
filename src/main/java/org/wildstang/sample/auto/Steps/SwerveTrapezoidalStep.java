package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.WsPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;



public class SwerveTrapezoidalStep extends AutoStep {

    private SwerveDrive swerve;
    private WsPose pose;
    private double turnStartTime; // Time to start turning to the end heading

    private Pose2d startPose;
    private Pose2d fieldAutoPose;
    private double angleToPoint;

    private Timer turnTimer;
    private Timer pathTimer;

    private WsTrapezoidalProfile profile;

    public SwerveTrapezoidalStep(SwerveDrive drive, Pose2d pose, double turnStart) {
        this.turnStartTime = turnStart;
        swerve = drive;
        turnTimer = new Timer();
        fieldAutoPose = pose;
    }

    public SwerveTrapezoidalStep(SwerveDrive drive, Pose2d pose) {
        this(drive, pose, 0);
    }

    @Override
    public void initialize() {
        swerve.setToAuto();
        turnTimer.start();
        pathTimer.start();
        profile = new WsTrapezoidalProfile(pose.estimatedPose.getTranslation().getDistance(fieldAutoPose.getTranslation()));
        WsPose pose = (WsPose) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_POSE);
        startPose = pose.estimatedPose;
        angleToPoint = Math.atan2(fieldAutoPose.getY() - startPose.getY(), fieldAutoPose.getX() - startPose.getX());
    }

    @Override
    public void update() {
        WsTrapezoidalProfile.State state = profile.calculateState(pathTimer.get());
        double xVelocity = state.velocity() * -Math.sin(angleToPoint); // Negative so positive angle is negative velocity
        double yVelocity = state.velocity() * Math.cos(angleToPoint);
        double xAcceleration = state.acceleration() * -Math.sin(angleToPoint); // Negative so positive angle is negative velocity
        double yAcceleration = state.acceleration() * Math.cos(angleToPoint); // Negative so positive angle is negative velocity
        Pose2d position = startPose.plus(new Transform2d(startPose, fieldAutoPose).times(state.distance() / profile.getDistance()));

        if (pathTimer.hasElapsed(profile.getFinalTIme())) {

        }

        if (turnTimer.hasElapsed(turnStartTime)) {
            swerve.setAutoValues(xVelocity,yVelocity,xAcceleration,yAcceleration, position);
        } else {
            swerve.setAutoValues(xVelocity,yVelocity,xAcceleration,yAcceleration,new Pose2d(position.getTranslation(), swerve.odoAngle()));
        }
    }

    @Override
    public String toString() {
        return "Trapezoidal Swerve To Point Step";
    }
}
