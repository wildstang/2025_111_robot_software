package org.wildstang.sample.subsystems.targeting;

import java.util.List;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionConsts {

    public static final double inToM = 1/39.37;
    public final double mToIn = 39.37;

    // All Variables in Blue coordinate system
    public static final List<Pose2d> leftTopBranches = List.of(
        new Pose2d(3.75, 2.95, new Rotation2d(Math.toRadians(120))), //front right
        new Pose2d(3.9, 5.2, new Rotation2d(Math.toRadians(240))), //front left
        new Pose2d(5.80, 3.925, new Rotation2d(Math.toRadians(0))));//back
    public static final List<Pose2d> rightTopBranches = List.of(
        new Pose2d(4.06, 2.78, new Rotation2d(Math.toRadians(120))), //front right
        new Pose2d(3.66, 5.0, new Rotation2d(Math.toRadians(240))), //front left
        new Pose2d(5.785, 4.28, new Rotation2d(Math.toRadians(0))));//back
    public static final List<Pose2d> leftBottomBranches = List.of(
        new Pose2d(3.18, 4.125, new Rotation2d(Math.toRadians(180))), //front
        new Pose2d(5.218, 5.10, new Rotation2d(Math.toRadians(300))), //back left
        new Pose2d(5.05, 2.84, new Rotation2d(Math.toRadians(60))));//back right
    public static final List<Pose2d> rightBottomBranches = List.of(
        new Pose2d(3.218, 3.78, new Rotation2d(Math.toRadians(180))), //front
        new Pose2d(4.911, 5.27, new Rotation2d(Math.toRadians(300))), //back left
        new Pose2d(5.335, 3.026, new Rotation2d(Math.toRadians(60))));//back right

    public static final Translation2d reefCenter = new Translation2d(176*inToM, 158.5*inToM);
    public static final double halfwayAcrossFieldY = (317/2)*inToM;
    public static final double coralStationLeftHeading = 235;
    public static final double coralStationRightHeading = 125;

    // X value of the translation is irrelevant
    public static final Translation2d netScore = new Translation2d(300*inToM, 300*inToM);

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