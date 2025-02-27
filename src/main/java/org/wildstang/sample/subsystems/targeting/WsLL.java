package org.wildstang.sample.subsystems.targeting;

import java.util.Optional;

import org.wildstang.framework.core.Core;
import org.wildstang.sample.subsystems.targeting.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WsLL {

    private final double mToIn = 39.3701;

    private PoseEstimate blue3D;
    private PoseEstimate red3D;
    public PoseEstimate alliance3D;
    public int tid;
    public boolean tv;
    public double tx;
    public double ty;
    public double tl;
    public double tc;
    public double ta;
    StructPublisher<Pose2d> posePublisher;

    // Name of Limelight
    public String CameraID;
    public boolean isAprilTag;

    /*
     * Argument is String ID of the limelight networktable entry, aka what it's called
     */
    public WsLL(String CameraID, boolean isAprilTag){
        posePublisher = NetworkTableInstance.getDefault().getStructTopic(CameraID + "/metatag2 alliance pose", Pose2d.struct).publish();
        
        tv = LimelightHelpers.getTV(CameraID); // 1 if valid target exists. 0 if no valid targets exist
        tx = LimelightHelpers.getTX(CameraID); // Horrizontal offset from crosshair to target
        ty = LimelightHelpers.getTY(CameraID); // Vertical offset from crosshair to target
        tl = LimelightHelpers.getLatency_Pipeline(CameraID); // The pipeline's latency contribution (ms)
        tc = LimelightHelpers.getLatency_Capture(CameraID); // Capture pipeline latency (ms)
        ta = LimelightHelpers.getTA(CameraID); // Target area

        if (isAprilTag) {
            tid = (int) LimelightHelpers.getFiducialID(CameraID); // ID of the primary in view april tag
            blue3D = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CameraID);
            red3D = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(CameraID);
            alliance3D = Core.isBlue() ? blue3D : red3D;
            if (alliance3D != null){
                posePublisher.set(alliance3D.pose);
            }
        }

        this.CameraID = CameraID;
        this.isAprilTag = isAprilTag;
    }

    // Limelight NT botpose array values
    // 0 translation X
    // 1 translation Y
    // 2 translation Z
    // 3 rotation roll (degrees)
    // 4 rotation pitch
    // 5 rotation  yaw
    // 6 total latency (ms)
    // 7 tag count
    // 8 tag span
    // 9 average tag distance from camera
    // 10 average tag area (percentage of image)

    /**
     * Updates all values to the latest value
     * @param yaw Yaw value for MegaTag2 pose disambiguation
     * @return pose estimate if valid
     */
    public Optional<PoseEstimate> update(double yaw){
        LimelightHelpers.SetRobotOrientation(CameraID, yaw, 0, 0, 0, 0, 0); 
        
        tid = (int) LimelightHelpers.getFiducialID(CameraID); // ID of the primary in view april tag
        tv = LimelightHelpers.getTV(CameraID); // 1 if valid target exists. 0 if no valid targets exist
        tx = LimelightHelpers.getTX(CameraID); // Horrizontal offset from crosshair to target
        ty = LimelightHelpers.getTY(CameraID); // Vertical offset from crosshair to target
        tl = LimelightHelpers.getLatency_Pipeline(CameraID); // The pipeline's latency contribution (ms)
        tc = LimelightHelpers.getLatency_Capture(CameraID); // Capture pipeline latency (ms)
        ta = LimelightHelpers.getTA(CameraID); // Target area
        if (tv && isAprilTag ){
            double oldTimestamp = alliance3D != null ? alliance3D.timestampSeconds : Double.MAX_VALUE;
            blue3D = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CameraID);
            red3D = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(CameraID);
            alliance3D = Core.isBlue() ? blue3D : red3D;
            if (alliance3D != null) posePublisher.set(alliance3D.pose);

            // We don't actually have a new frame and MegTag2 pose to return
            if (alliance3D != null && alliance3D.timestampSeconds == oldTimestamp) {
                return Optional.empty();
            }
            if (alliance3D != null) return Optional.of(alliance3D);
            else return Optional.empty();
        }
        return Optional.empty();
    }
    /*
     * returns true if a target is seen, false otherwise
     */
    public boolean TargetInView(){
        return tv;
    }
    
    /*
     * returns total latency, capture latency + pipeline latency
     */
    public double getTotalLatency(){
        return tc + tl;
    }
    /*
     * Sets the pipeline (0-9) with argument
     */
    public void setPipeline(int pipeline){
        LimelightHelpers.setPipelineIndex(CameraID, pipeline);
    }


    public enum LEDState { DEFAULT, OFF, BLINK, ON}
    /*
     * Sets what the LED lights do
     */
    public void setLED(LEDState state){
        switch (state) {
            case DEFAULT:
                LimelightHelpers.setLEDMode_PipelineControl(CameraID);
            case OFF:
                LimelightHelpers.setLEDMode_ForceOff(CameraID);
            case BLINK:
                LimelightHelpers.setLEDMode_ForceBlink(CameraID);
            case ON:
                LimelightHelpers.setLEDMode_ForceOn(CameraID);
        }
    }
}