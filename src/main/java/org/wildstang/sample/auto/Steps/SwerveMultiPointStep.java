package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;



public class SwerveMultiPointStep extends AutoStep {

    private final double startingPower = 0.5;//initial power limit at the start
    private final double timeToMaxSpeed = 0.25;//time until full speed

    private SwerveDrive swerve;

    private int index = 0;
    private Pose2d[] poses;
    private double[] speeds;
    private double turnStartTime; // Time to start turning to the end heading


    private Timer timer = new Timer();

    /**
     * Drives through multiple points at full speed for intermediate points or limit given by speeds array
     * Uses P loop for final point
     * Threshold for atPosition for intermediate poses is lower
     * @param poses list of poses to drive through
     * @param speeds speeds limts to get to each pose
     * @param turnstart time to delay turning to the pose
     */
    public SwerveMultiPointStep(Pose2d[] poses, double[] speeds, double turnstart) {
        this.turnStartTime = turnstart;
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        this.poses = poses;
        this.speeds = speeds;
    }

    public SwerveMultiPointStep(Pose2d[] poses, double[] speeds) {
        this(poses, speeds, 0.0);
    }

    @Override
    public void initialize() {
        swerve.setToAuto();
        timer.start();
    }

    @Override
    public void update() {

        swerve.setAutoValues(0,0,0,0,new Pose2d(poses[index].getTranslation(), swerve.odoAngle()));

        // Drive to intermediate point
        if (index < poses.length - 1) {
            swerve.usePID(false);
            if (swerve.isAtPosition(0.2)) {
                index++;
            }
        } else {
            swerve.usePID(true);
            if (swerve.isAtPosition()) {
                setFinished();
                return;
            }
            swerve.usePID(true);
        }
        if (timer.hasElapsed(turnStartTime)) {
            swerve.setAutoValues(0,0,0.0,0.0, poses[index]);
        } else {
            swerve.setAutoValues(0,0,0,0,new Pose2d(poses[index].getTranslation(), swerve.odoAngle()));
        }

        // Limit power by acceleration limiter or speeds value for that part of the path, if no speed in array then don't limit
        swerve.setAutoScalar(Math.min(startingPower + timer.get() * (1 - startingPower)/(timeToMaxSpeed), index < speeds.length ? speeds[index] : 1));
    }

    @Override
    public String toString() {
        return "Swerve Multi Point Step";
    }
}
