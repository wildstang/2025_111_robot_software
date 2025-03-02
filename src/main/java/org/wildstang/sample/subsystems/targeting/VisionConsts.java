package org.wildstang.sample.subsystems.targeting;

import java.util.List;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionConsts {

    public static final double inToM = 1/39.37;
    public final double mToIn = 39.37;

    // All Variables in Blue coordinate system
    public static final List<Pose2d> leftBranches = List.of(
        new Pose2d(3.19, 4.12, new Rotation2d(Math.toRadians(180))), //front
        new Pose2d(5.24, 5.10, new Rotation2d(Math.toRadians(300))), //back left
        new Pose2d(5.05, 2.84, new Rotation2d(Math.toRadians(60))),//back right
        new Pose2d(3.75, 2.95, new Rotation2d(Math.toRadians(120))), //front right
        new Pose2d(3.93, 5.21, new Rotation2d(Math.toRadians(240))), //front left
        new Pose2d(5.79, 3.93, new Rotation2d(Math.toRadians(0))));//back
    public static final List<Pose2d> rightBranches = List.of(
        new Pose2d(3.20, 3.80, new Rotation2d(Math.toRadians(180))), //front
        new Pose2d(4.93, 5.27, new Rotation2d(Math.toRadians(300))), //back left
        new Pose2d(5.35, 3.03, new Rotation2d(Math.toRadians(60))),//back right
        new Pose2d(4.05, 2.79, new Rotation2d(Math.toRadians(120))), //front right
        new Pose2d(3.64, 5.04, new Rotation2d(Math.toRadians(240))), //front left
        new Pose2d(5.78, 4.26, new Rotation2d(Math.toRadians(0))));//back
    public static final List<Pose2d> rightBranchL1 = List.of(
        new Pose2d(3.06, 3.57, new Rotation2d(Math.toRadians(189))), //front
        new Pose2d(5.02, 5.42, new Rotation2d(Math.toRadians(320))), //back left
        new Pose2d(5.37, 2.93, new Rotation2d(Math.toRadians(82))),//back right
        new Pose2d(4.08, 2.58, new Rotation2d(Math.toRadians(133))), //front right
        new Pose2d(3.64, 5.15, new Rotation2d( Math.toRadians(266))),//front left
        new Pose2d(5.90, 4.28, new Rotation2d(Math.toRadians(19))));//back
    public static final List<Pose2d> leftBranchL1 = List.of(
        new Pose2d(3.08, 4.08, new Rotation2d(Math.toRadians(155))), //front
        new Pose2d(5.25, 5.20, new Rotation2d(Math.toRadians(278))), //back left
        new Pose2d(5.00, 2.74, new Rotation2d(Math.toRadians(48))),//back right
        new Pose2d(3.76, 2.82, new Rotation2d(Math.toRadians(95))), //front right
        new Pose2d(4.02, 5.31, new Rotation2d(Math.toRadians(230))), //front left
        new Pose2d(5.87, 3.87, new Rotation2d(Math.toRadians(344))));//back

    // HOME CALIBRATION
    // // All Variables in Blue coordinate system
    // public static final List<Pose2d> leftBranches = List.of(
    //     new Pose2d(3.18, 4.125, new Rotation2d(Math.toRadians(180))), //front
    //     new Pose2d(5.218, 5.10, new Rotation2d(Math.toRadians(300))), //back left
    //     new Pose2d(5.05, 2.84, new Rotation2d(Math.toRadians(60))),//back right
    //     new Pose2d(3.75, 2.95, new Rotation2d(Math.toRadians(120))), //front right
    //     new Pose2d(3.9, 5.2, new Rotation2d(Math.toRadians(240))), //front left
    //     new Pose2d(5.80, 3.925, new Rotation2d(Math.toRadians(0))));//back
    // public static final List<Pose2d> rightBranches = List.of(
    //     new Pose2d(3.218, 3.78, new Rotation2d(Math.toRadians(180))), //front
    //     new Pose2d(4.911, 5.27, new Rotation2d(Math.toRadians(300))), //back left
    //     new Pose2d(5.335, 3.026, new Rotation2d(Math.toRadians(60))),//back right
    //     new Pose2d(4.06, 2.78, new Rotation2d(Math.toRadians(120))), //front right
    //     new Pose2d(3.66, 5.0, new Rotation2d(Math.toRadians(240))), //front left
    //     new Pose2d(5.785, 4.28, new Rotation2d(Math.toRadians(0))));//back
    // public static final List<Pose2d> rightBranchL1 = List.of(
    //     new Pose2d(3.06, 3.73, new Rotation2d(Math.toRadians(200))), //front
    //     new Pose2d(4.946, 5.43, new Rotation2d(Math.toRadians(320))), //back left
    //     new Pose2d(5.455, 2.916, new Rotation2d(Math.toRadians(80))),//back right
    //     new Pose2d(4.03, 2.62, new Rotation2d(Math.toRadians(140))), //front right
    //     new Pose2d(3.54, 5.11, new Rotation2d( Math.toRadians(260))),//front left
    //     new Pose2d(5.94, 4.33, new Rotation2d(Math.toRadians(20))));//back
    // public static final List<Pose2d> leftBranchL1 = List.of(
    //     new Pose2d(3.18, 4.125, new Rotation2d(Math.toRadians(160))), //front
    //     new Pose2d(5.218, 5.10, new Rotation2d(Math.toRadians(280))), //back left
    //     new Pose2d(5.05, 2.84, new Rotation2d(Math.toRadians(40))),//back right
    //     new Pose2d(3.75, 2.95, new Rotation2d(Math.toRadians(100))), //front right
    //     new Pose2d(3.9, 5.2, new Rotation2d(Math.toRadians(220))), //front left
    //     new Pose2d(5.80, 3.925, new Rotation2d(Math.toRadians(340))));//back
    

    public static final Translation2d reefCenter = new Translation2d(176*inToM, 158.5*inToM);
    public static final double halfwayAcrossFieldY = (317/2)*inToM;
    public static final double coralStationLeftHeading = 235;
    public static final double coralStationRightHeading = 125;

    // X value of the translation is irrelevant
    public static final Translation2d netScore = new Translation2d(295*inToM, 295*inToM);

    public static final int ATPipelineIndex = 0;

    /*
     * April Tag IDs:
     *  -Red Coral stations: left 1, right 2
     *  -Red Processor 3
     *  -Red side of barge: blue barge 4, red barge 5
     *  -Red reef: FL 6, F 7, FR 8, BR 9, B 10, BL 11
     *  -Blue Coral Stations: right 12, left 13
     *  -Blue side of barge: blue barge 14, red barge 15
     *  -Blue processor: 16
     *  -Blue reef: FR 17, F 18, FL 19, BL 20, B 21, BR 22
     */

    //public static Pose3d cameraPose = new Pose3d(-11.8*inToM,8.5*inToM,22.3*inToM, new Rotation3d(0.0,-65*Math.PI/180.0,11*Math.PI/180.0));

}