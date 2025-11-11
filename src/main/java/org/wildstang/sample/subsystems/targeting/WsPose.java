package org.wildstang.sample.subsystems.targeting;

// ton of imports
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.DriveConstants;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.LimelightHelpers.PoseEstimate;
import org.wildstang.sample.subsystems.targeting.LimelightHelpers.RawFiducial;

import java.util.Optional;

import org.wildstang.framework.core.Core;

import org.wildstang.framework.io.inputs.Input;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.io.IOException;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.ImmutableEnergy;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.wildstang.sample.subsystems.targeting.FOMConstants;

public class WsPose implements Subsystem {

    private WsAprilTagLL left;
    private WsAprilTagLL right;
    private WsAprilTagLL front;

    private double distanceDriven = 0;
    

    private WsAprilTagLL[] cameras; 

    // Object detection camera
    public WsGamePieceLL object = new WsGamePieceLL("limelight-object");

    private final double poseBufferSizeSec = 2;
    public final double visionSpeedThreshold = 3.0;
    
    public int currentID = 0;
    public SwerveDrive swerve;

    // WPI blue relative (m and CCW rad)
    public Pose2d odometryPose = new Pose2d();
    public Pose2d estimatedPose = new Pose2d();

    StructPublisher<Pose2d> odometryPosePublisher = NetworkTableInstance.getDefault().getStructTopic("odometryPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedPosePublisher = NetworkTableInstance.getDefault().getStructTopic("estimatedPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> coralPosePublisher = NetworkTableInstance.getDefault().getStructTopic("coralPose", Pose2d.struct).publish();


    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private SwerveModulePosition[] lastWheelPositions = {};
    private Rotation2d lastGyroAngle = new Rotation2d();


    private double oldOdometryUpdateTime = 0.0;
    @Override
    public void inputUpdate(Input source) {
    }

    @Override
    public void initSubsystems() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        left = new WsAprilTagLL("limelight-left", swerve::getMegaTag2Yaw);
        right = new WsAprilTagLL("limelight-right", swerve::getMegaTag2Yaw);
        front = new WsAprilTagLL("limelight-object", swerve::getMegaTag2Yaw);
        cameras = new WsAprilTagLL[] {left, right, front};
    }

    @Override
    public void init() {
    }

    @Override
    public void selfTest() {
    }

    
    @Override
    public void update() {
        object.update();
        
            double bestStdDev = Double.MAX_VALUE;
            PoseEstimate bestEstimate = null;

            WsAprilTagLL bestCamera = getBestCamera();
            
        
            if(bestCamera == null){
                estimatedPose = odometryPose;
                SmartDashboard.putString("Vision/BestCamera", "none");
                odometryPosePublisher.set(odometryPose);
                odometryPosePublisher.set(estimatedPose);
                return;
            }
            

            bestEstimate = bestCamera.update().orElse(null);
            double odFOM = odometryFOM();
            SmartDashboard.putNumber("Odometry FOM", odFOM);

        if(bestEstimate == null){

            estimatedPose = odometryPose;
            SmartDashboard.putString("Vision/BestCamera", bestCamera.CameraID);
        }else{
            bestStdDev = getStdDev(Optional.of(bestEstimate));
            double camFOM = cameraFOM(bestCamera);
           

            SmartDashboard.putNumber("Camera FOM", camFOM);
            SmartDashboard.putNumber("Best StdDev", bestStdDev);

            if(camFOM > odFOM){
          
                estimatedPose = odometryPose;
                
            
            }else if(camFOM < odFOM){
                /*for (int i = 0; i < cameras.length; i++) {
                    Optional<PoseEstimate> estimate = cameras[i].update();
                    if (estimate.isPresent() && getStdDev(estimate) < bestStdDev) {
                        bestEstimate = estimate.get();
                        bestStdDev = getStdDev(estimate);
                    }   
                }*/
                addVisionObservation(bestEstimate, 1/bestStdDev);
            }
        }

        odometryPosePublisher.set(odometryPose);
        estimatedPosePublisher.set(estimatedPose);
        
        SmartDashboard.putNumber("Best Standard Deviation ", bestStdDev);
        if(bestEstimate != null){
            SmartDashboard.putNumberArray("Best Estimate", new double[]{bestEstimate.pose.getX(), bestEstimate.pose.getY()});
        }else if (estimatedPose != null){
            SmartDashboard.putNumberArray("Estimated Pose", new double[]{estimatedPose.getX(), estimatedPose.getY()});
        }
       
        
    }
    



    private double cameraFOM(WsAprilTagLL bestCamera){
        double robotSpeed = swerve.speedMagnitude();
        SmartDashboard.putNumber("Robot Speed", robotSpeed);
        double tempSpeed = Math.abs(swerve.speeds().omegaRadiansPerSecond);
        double rotSpeed = tempSpeed;
        if(tempSpeed < 0){
            rotSpeed *= -1;
        }
        SmartDashboard.putNumber("Temp Speed", tempSpeed);
        SmartDashboard.putNumber("Robot Rotation Speed", rotSpeed);
        return (robotSpeed * FOMConstants.ROBOT_SPEED) + (rotSpeed * FOMConstants.ROT_SPEED);
    }

    

    private double odometryFOM(){

        double robotSpeed = swerve.speedMagnitude();
        
        double newTime = Timer.getFPGATimestamp();
        double deltaT = newTime - oldOdometryUpdateTime;
        oldOdometryUpdateTime = newTime;

        distanceDriven += 0.1*(Math.abs(robotSpeed)*deltaT);
        SmartDashboard.putNumber("Distance Driven", distanceDriven);


        return (Math.abs(robotSpeed) * deltaT) * FOMConstants.ODOMETRY_DISPLACEMENT + distanceDriven;
    }

    private WsAprilTagLL getBestCamera(){

        if(LimelightHelpers.getTargetCount(left.CameraID) == 0 && LimelightHelpers.getTargetCount(right.CameraID) == 0){
            SmartDashboard.putNumber("Priority tag",0);
            return null;
        }
        Optional<PoseEstimate> leftEstimate = left.update(); 
        Optional<PoseEstimate> rightEstimate = right.update();
        
        //checking if any of the camera poses are null
       if(!leftEstimate.isPresent() && !rightEstimate.isPresent()){
           return null;
       }

       SmartDashboard.putBoolean("sees right estimate?", rightEstimate.isPresent());
        if(!leftEstimate.isPresent() && rightEstimate.isPresent()){
            SmartDashboard.putNumber("Priority tag",1);
            return right;
        }
        else if(!rightEstimate.isPresent() && leftEstimate.isPresent()){
            SmartDashboard.putNumber("Priority tag",2);
            return left;
        }

        int lID = left.tid;
        int rID = right.tid;

            SmartDashboard.putNumber("Priority tag",3);
            // Get distance from priority tag to each camera
            /*
             * Calcualte the distance by getting camera pose coordinates and the coordinates of priority tag and get distance
             * 
             */
                double leftDistance = 0;
                double rightDistance = 0;
               
            
                RawFiducial[] arrayOfTagsForLeftCamera = LimelightHelpers.getRawFiducials(left.CameraID);
                RawFiducial[] arrayOfTagsForRightCamera = LimelightHelpers.getRawFiducials(right.CameraID);

                for(int i = 0; i < arrayOfTagsForLeftCamera.length; i++ ){
                    if(arrayOfTagsForLeftCamera[i].id == lID){
                        leftDistance = arrayOfTagsForLeftCamera[i].distToCamera;
                    }
                }
                for(int i = 0; i < arrayOfTagsForRightCamera.length; i++ ){
                    if(arrayOfTagsForRightCamera[i].id == rID){
                        rightDistance = arrayOfTagsForRightCamera[i].distToCamera;
                    }
                }
                if (leftDistance < rightDistance){
                    SmartDashboard.putNumber("Priority tag",4);
                    return left;
                }else if(leftDistance > rightDistance){
    
                    SmartDashboard.putNumber("Priority tag",5);
                    return right;
                } 
    
        return null;


    }



    public double getStdDev(Optional<PoseEstimate> bestEstimate) {
            return bestEstimate.isPresent() && bestEstimate.get().rawFiducials.length > 0 ? Math.pow(Arrays.stream(bestEstimate.get().rawFiducials).mapToDouble(fiducial -> fiducial.distToCamera).min().getAsDouble(),2) / bestEstimate.get().tagCount : Double.MAX_VALUE;
    }

    @Override
    public void resetState() {
        left.update();
        right.update();
    }

    /**
     * Reset estimated pose and odometry pose to pose
     * Clear pose buffer
    */
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle) {
        
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
        poseBuffer.addSample(Timer.getTimestamp(), odometryPose);
        estimatedPose = estimatedPose.exp(twist);
    }

    private void addVisionObservation(PoseEstimate bestEstimate, double weight) {
            
            SmartDashboard.putNumber("auto weight", weight);
            Optional<Pose2d> sample = poseBuffer.getSample(bestEstimate.timestampSeconds);
            if (sample.isEmpty()) {
                // exit if not there
                return;
            }
    
            // sample --> odometryPose transform and backwards of that
            var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose); // current bservatin 
            var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    
            // get old estimate by applying odometryToSample Transform
            Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
      // difference between estimate and vision pose
          
            Transform2d transform = new Transform2d(estimateAtTime, bestEstimate.pose);
        transform = transform.times(Math.max(1, weight));

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(transform).plus(sampleToOdometryTransform);
    }

    // YEAR SUBSYSTEM ACCESS METHODS


    /**
     * Can the object detection camera see a coral
     * @return true if a coral is present
     */
    public boolean coralInView(){
        return object.targetInView();
    }
    /**
     * Coral pose
     * 
     * @return field relative pose of the coral if the coral is present
     */
    public Optional<Translation2d> getCoralPose() {
        if (!object.targetInView()) return Optional.empty();

        Optional<Pose2d> sample = poseBuffer.getSample(object.timestamp);
        if (sample.isEmpty()) {
            // exit if not there
            return Optional.empty();
        }

        // current odometryPose --> sample transformation
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

        // get old estimate at timestamp by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Assumes the angle of depression is gonna be negative
        double camToCoralDist = VisionConsts.camTransform.getZ() / -Math.tan(VisionConsts.camTransform.getRotation().getY() + Math.toRadians(object.ty));

        // Transform from camera to coral
        Transform2d camToCoralTransform = new Transform2d(Math.cos(Math.toRadians(-object.tx)) * camToCoralDist, Math.sin(Math.toRadians(-object.tx)) * camToCoralDist, Rotation2d.fromDegrees(-object.tx));

        Transform2d camTransform2d = new Transform2d(VisionConsts.camTransform.getX(), VisionConsts.camTransform.getY(), new Rotation2d(VisionConsts.camTransform.getRotation().getZ()));
        // Combines transformations to get coral pose in field coordinates
        return Optional.of(estimateAtTime.plus(camTransform2d).plus(camToCoralTransform).getTranslation());
    }

    // Set the front limlelight (same camera referenced by both front and object) to object or april tag pipeline
    public void setPipelineObject(boolean isObject) {
        if (isObject) {
            object.setPipeline(0);
        } else {
            front.setPipeline(1);
        }
    }

    /**
     * Based on whether we are scoring left branch or right branch gets the closest scoring pose
     * @param right True if we are scoring on the right branch
     * @return Pose to align the robot to to score
     */
    public Pose2d getClosestBranch(boolean right) {
        if (right) {
            return estimatedPose.nearest(VisionConsts.rightBranches);
        } else {
            return estimatedPose.nearest(VisionConsts.leftBranches);
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
     * @return true if we are on the net side of the field and should be scoring in the net if we have algae
     */
    public boolean isAlgaeScoreNet() {
        return estimatedPose.getTranslation().getY() > 3.7;
    }

    /**
     * Returns if we are near the reef and can start raising lift to staged height
     * @return true if we are within 2.5 m of the reef center
     */
    public boolean nearReef() {
        return estimatedPose.getTranslation().getDistance(VisionConsts.reefCenter) < 2.5;
    }

    /**
     * @param target WPI blue target coordinate (m) to align with proportional control loop
     * @return Control value for Driver Station relative X power for aligning robot to certain target
     */
    public double getAlignX(Translation2d target) {
        return DriveConstants.ALIGN_P * -(target.getY() - estimatedPose.getY());
    }

    /**
     * @param target WPI blue target coordinate (m) to align with proportional control loop
     * @return Control value for Driver Station relative Y power for aligning robot to certain target
     */
    public double getAlignY(Translation2d target) {
        return DriveConstants.ALIGN_P * (target.getX() - estimatedPose.getX());
    }
    
    /**
     * @param target WPI blue Target coordinate (m)
     * @return Controller bearing degrees (aka what to plug into rotTarget)
     */
    public double turnToTarget(Translation2d target) {
        double offsetX = target.getX() - estimatedPose.getX();
        double offsetY = target.getY() - estimatedPose.getY();
        return (360 -Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
    }


    @Override
    public String getName() {
        return "Ws Pose";
    }
}