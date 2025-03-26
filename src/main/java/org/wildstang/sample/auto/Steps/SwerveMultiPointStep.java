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

    private int index;
    private Pose2d[] poses;
    private double[] speeds;
    private double turnStartTime; // Time to start turning to the end heading


    private Timer timer = new Timer();

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


        // Drive to intermediate point
        if (index < poses.length - 1) {
            swerve.usePID(false);
            if (swerve.isAtPosition(0.2)) {
                index++;
            }
        } else {
            if (swerve.isAtPosition()) {
                setFinished();
                return;
            }
            swerve.usePID(true);
        }
        if (timer.hasElapsed(turnStartTime)) {
            swerve.setAutoValues(0.0,0.0, poses[index]);
        } else {
            swerve.setAutoValues(0,0,new Pose2d(poses[index].getTranslation(), swerve.odoAngle()));
        }
        swerve.setAutoScalar(Math.min(startingPower + timer.get() * (1 - startingPower)/(timeToMaxSpeed), speeds[index]));


        // if (timer.hasElapsed(turnStartTime)) {
        //     swerve.setAutoValues(0.0,0.0, fieldAutoPose);
        // } else {
        //     swerve.setAutoValues(0,0,new Pose2d(fieldAutoPose.getTranslation(), swerve.odoAngle()));
        // }
        // swerve.setAutoScalar(startingPower + timer.get() * (1 - startingPower)/(timeToMaxSpeed));
        // if (swerve.isAtPosition()) {
        //     swerve.setAutoScalar(2.0);
        //     setFinished();
        // } 
    }

    @Override
    public String toString() {
        return "Swerve To Point Step";
    }
}
