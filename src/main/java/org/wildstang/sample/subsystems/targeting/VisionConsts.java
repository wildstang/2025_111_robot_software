package org.wildstang.sample.subsystems.targeting;

import java.util.List;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionConsts {

    public static final double inToM = 1/39.71;
    public final double mToIn = 39.71;

    // All Variables in Blue coordinate system
    public static final List<Pose2d> leftTopBranches = List.of(
        new Pose2d(145*inToM, 124*inToM, new Rotation2d(Math.toRadians(120))), 
        new Pose2d(163*inToM, 203*inToM, new Rotation2d(Math.toRadians(240))), 
        new Pose2d(222*inToM, 148*inToM, new Rotation2d(Math.toRadians(0))));
    public static final List<Pose2d> rightTopBranches = List.of(
        new Pose2d(162*inToM, 113*inToM, new Rotation2d(Math.toRadians(120))), 
        new Pose2d(145*inToM, 193*inToM, new Rotation2d(Math.toRadians(240))), 
        new Pose2d(223*inToM, 169*inToM, new Rotation2d(Math.toRadians(0))));
    public static final List<Pose2d> leftBottomBranches = List.of(
        new Pose2d(132*inToM, 169*inToM, new Rotation2d(Math.toRadians(180))), 
        new Pose2d(208*inToM, 193*inToM, new Rotation2d(Math.toRadians(300))), 
        new Pose2d(191*inToM, 114*inToM, new Rotation2d(Math.toRadians(60))));
    public static final List<Pose2d> rightBottomBranches = List.of(
        new Pose2d(131*inToM, 148*inToM, new Rotation2d(Math.toRadians(180))), 
        new Pose2d(191*inToM, 204*inToM, new Rotation2d(Math.toRadians(300))), 
        new Pose2d(209*inToM, 124*inToM, new Rotation2d(Math.toRadians(60))));
    public static final Translation2d reefCenter = new Translation2d(176*inToM, 158.5*inToM);
    public static final double halfwayAcrossFieldY = (317/2)*inToM;
    // TODO: what are the actual values?
    public static final double coralStationLeftHeading = 225;
    public static final double coralStationRightHeading = 135;

    // X value of the translation is irrelevant
    public static final Translation2d netScore = new Translation2d(300*inToM, 300*inToM);

    //TODO: what are the actual pipeline indices?
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