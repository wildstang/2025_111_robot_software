package org.wildstang.sample.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.drive.Drive;
import org.wildstang.sample.subsystems.swerve.DriveConstants;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.LimelightHelpers.PoseEstimate;
import org.wildstang.sample.subsystems.targeting.LimelightHelpers.RawFiducial;

import java.util.Optional;

import org.wildstang.framework.core.Core;

import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.Arrays;
import java.util.stream.DoubleStream;
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

    private final double poseBufferSizeSec = 2;
    public final double visionSpeedThreshold = 3.0;
    
    public int currentID = 0;
    public SwerveDrive swerve;

    private boolean isInAuto = false;

    // Always field relative (m and CCW rad)
    public Pose2d odometryPose = new Pose2d();
    public Pose2d estimatedPose = new Pose2d();

    StructPublisher<Pose2d> odometryPosePublisher;
    StructPublisher<Pose2d> estimatedPosePublisher;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private SwerveModulePosition[] lastWheelPositions = {};
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

    @Override
    public void update() {
        Optional<PoseEstimate> leftEstimate = left.update(swerve.getMegaTag2Yaw());
        Optional<PoseEstimate> rightEstimate = right.update(swerve.getMegaTag2Yaw());

        // Same standard deviation calculation as 6328 but only used for calculating validity of vision estimate
        double leftStdDev = Double.MAX_VALUE;
        double rightStdDev = Double.MAX_VALUE;
        if (swerve.speedMagnitude() < visionSpeedThreshold && 
            (!isInAuto || (estimatedPose.getTranslation().getDistance(VisionConsts.reefCenter) < 3))) {
            if (leftEstimate.isPresent() && leftEstimate.get().rawFiducials.length > 0) {

                // Get distance to the closest tag from the array of raw fiducials
                double closestTagDist = Arrays.stream(leftEstimate.get().rawFiducials).mapToDouble(fiducial -> fiducial.distToCamera).min().getAsDouble();
                leftStdDev = Math.pow(closestTagDist, 2) / leftEstimate.get().tagCount;
                if (leftEstimate.get().avgTagDist > 3.5) leftStdDev = Double.MAX_VALUE;
            }
            if (rightEstimate.isPresent() && rightEstimate.get().rawFiducials.length > 0) {

                // Get distance to the closest tag from the array of raw fiducials
                double closestTagDist = Arrays.stream(rightEstimate.get().rawFiducials).mapToDouble(fiducial -> fiducial.distToCamera).min().getAsDouble();
                rightStdDev = Math.pow(closestTagDist, 2) / rightEstimate.get().tagCount;
                if (rightEstimate.get().avgTagDist > 3.5) rightStdDev = Double.MAX_VALUE;
            }
            if (leftStdDev < rightStdDev) {
                addVisionObservation(leftEstimate.get());
                currentID = left.tid;
            } else if (rightStdDev < leftStdDev) {
                addVisionObservation(rightEstimate.get());
                currentID = right.tid;
            }
        }

        odometryPosePublisher.set(odometryPose);
        estimatedPosePublisher.set(estimatedPose);
    }

    @Override
    public void resetState() {
        left.update(swerve.getMegaTag2Yaw());
        right.update(swerve.getMegaTag2Yaw());
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

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, boolean isAuto) {
        if (lastWheelPositions.length == 0) { 
            lastWheelPositions = modulePositions;
            return; 
        }
        Twist2d twist = DriveConstants.kinematics.toTwist2d(lastWheelPositions, modulePositions);
        twist.dtheta = gyroAngle.minus(lastGyroAngle).getRadians();
        odometryPose = odometryPose.exp(twist);
        
        lastWheelPositions = modulePositions;
        lastGyroAngle = gyroAngle;
        // Add pose to buffer at timestamp
        poseBuffer.addSample(Timer.getFPGATimestamp(), odometryPose);

        estimatedPose = estimatedPose.exp(twist);
        isInAuto = isAuto;
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

    // YEAR SUBSYSTEM ACCESS METHODS

    /**
     * Based on whether we are scoring left branch or right branch gets the closest scoring pose
     * @param right // true if we are scoring on the right branch
     * @return pose to align the robot to to score
     */
    public Pose2d getClosestBranch(boolean right) {
        if (right) {
            return estimatedPose.nearest(VisionConsts.rightBranches);
        } else {
            return estimatedPose.nearest(VisionConsts.leftBranches);
        }
    }

    /**
     * Based on whether we are scoring left branch or right branch gets the closest scoring pose
     * @param right // true if we are scoring on the right branch
     * @return pose to align the robot to to score
     */
    public Pose2d getClosestL1Branch(boolean right) {
        if (right) {
            return estimatedPose.nearest(VisionConsts.rightBranchL1);
        } else {
            return estimatedPose.nearest(VisionConsts.leftBranchL1);
        }
    }

    /**
     * Gets closest coral station to robot to determine what direction to lock heading
     * @return boolean true for right, false for left
     */
    public boolean isClosestStationRight() {
        if (estimatedPose.getY() > VisionConsts.halfwayAcrossFieldY) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Returns if we are near the reef and can start raising lift to staged height
     * @return true if we are within 2.5 m of the reef center
     */
    public boolean nearReef() {
        return estimatedPose.getTranslation().getDistance(VisionConsts.reefCenter) < 2.5;
    }

    /**
     * Driver Station relative
     * @param target // Target coordinate (m) to align with proportional control loop
     * @return // Control value for X power for aligning robot to certain target
     */
    public double getAlignX(Translation2d target) {
        return DriveConstants.ALIGN_P * -(target.getY() - estimatedPose.getY());
    }

    /**
     * Driver Station relative
     * @param target // Target coordinate (m) to align with proportional control loop
     * @return // Control value for X power for aligning robot to certain target
     */
    public double getAlignY(Translation2d target) {
        return DriveConstants.ALIGN_P * (target.getX() - estimatedPose.getX());
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