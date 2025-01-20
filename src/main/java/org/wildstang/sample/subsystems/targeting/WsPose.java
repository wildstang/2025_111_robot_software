package org.wildstang.sample.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.DriveConstants;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.LimelightHelpers.PoseEstimate;

import java.util.Optional;

import org.wildstang.framework.core.Core;

import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;

public class WsPose implements Subsystem {

    public WsLL left = new WsLL("limelight-left", true);
    public WsLL right = new WsLL("limelight-right", true);
    public WsLL back = new WsLL("limelight-back", false);

    private final double Align_P = 0.006;
    private final double poseBufferSizeSec = 2;
    public final double visionSpeedThreshold = 0.5;
    
    public SwerveDrive swerve;

    // Always field relative (m and CCW rad)
    public Pose2d odometryPose = new Pose2d();
    public Pose2d estimatedPose = new Pose2d();

    StructPublisher<Pose2d> odometryPosePublisher;
    StructPublisher<Pose2d> estimatedPosePublisher;


    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private SwerveModulePosition[] lastWheelPositions =
    new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
    };
    private Rotation2d lastGyroAngle = new Rotation2d();

    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void initSubsystems() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void init() {
        odometryPosePublisher = NetworkTableInstance.getDefault().getStructTopic("odometryPose", Pose2d.struct).publish();
        estimatedPosePublisher = NetworkTableInstance.getDefault().getStructTopic("estimatedPose", Pose2d.struct).publish();
    }

    @Override
    public void selfTest() {
    }

    public Pose2d getClosestBranch(boolean right) {
        if (right) {
            return estimatedPose.nearest(VisionConsts.rightBranches);
        } else {
            return estimatedPose.nearest(VisionConsts.rightBranches);
        }
    }

    @Override
    public void update() {
        Optional<PoseEstimate> leftEstimate = left.update(swerve.getFieldYaw());
        Optional<PoseEstimate> rightEstimate = right.update(swerve.getFieldYaw());
        double leftStdDev = Double.MAX_VALUE;
        double rightStdDev = Double.MAX_VALUE;
        if (swerve.speedMagnitude() < visionSpeedThreshold) {
            if (leftEstimate.isPresent()) {
                leftStdDev = Math.pow(leftEstimate.get().avgTagDist, 2) / leftEstimate.get().tagCount;
            }
            if (rightEstimate.isPresent()) {
                rightStdDev = Math.pow(rightEstimate.get().avgTagDist, 2) / rightEstimate.get().tagCount;
            }
            if (leftStdDev < rightStdDev) {
                addVisionObservation(leftEstimate.get());
            } else if (rightStdDev < leftStdDev) {
                addVisionObservation(rightEstimate.get());
            }
        }

        odometryPosePublisher.set(odometryPose);
        estimatedPosePublisher.set(estimatedPose);
        back.update(swerve.getFieldYaw());
    }

    @Override
    public void resetState() {
        left.update(swerve.getFieldYaw());
        right.update(swerve.getFieldYaw());
        back.update(swerve.getFieldYaw());
    }

    /**
     * Reset estimated pose and odometry pose to pose <br>
     * Clear pose buffer
    */
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
        Twist2d twist = DriveConstants.kinematics.toTwist2d(lastWheelPositions, modulePositions);
        twist.dtheta = gyroAngle.minus(lastGyroAngle).getRadians();
        lastWheelPositions = modulePositions;
        odometryPose = odometryPose.exp(twist);
        lastGyroAngle = gyroAngle;
        // Add pose to buffer at timestamp
        poseBuffer.addSample(Timer.getFPGATimestamp(), odometryPose);

        estimatedPose = estimatedPose.exp(twist);
    }

    public void addVisionObservation(PoseEstimate observation) {
        Optional<Pose2d> sample = poseBuffer.getSample(observation.timestampSeconds);
        if (sample.isEmpty()) {
            // exit if not there
            return;
        }

        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.pose);

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(transform).plus(sampleToOdometryTransform);
    }

    @Override
    public String getName() {
        return "Ws Vision";
    }

    /**
     * Driver Station relative
     * @param target // Target coordinate (m) to align with proportional control loop
     * @return // Control value for X power for aligning robot to certain target
     */
    public double getAlignX(Translation2d target) {
        return Align_P * -(target.getY() - estimatedPose.getY());
    }

    /**
     * Driver Station relative
     * @param target // Target coordinate (m) to align with proportional control loop
     * @return // Control value for X power for aligning robot to certain target
     */
    public double getAlignY(Translation2d target) {
        return Align_P * (target.getX() - estimatedPose.getX());
    }

    public double distanceToTarget(Translation2d target) {
        return Math.hypot(estimatedPose.getX() - target.getX(), estimatedPose.getY() - target.getY());
    }
    
    /**
     * @param target Target coordinate (m)
     * @return Controller bearing degrees (aka what to plug into rotTarget)
     */
    public double turnToTarget(Translation2d target) {
        double offsetX = target.getX() - estimatedPose.getX();
        double offsetY = target.getY() - estimatedPose.getY();
        return (360 -Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
    }
}