package org.wildstang.sample.subsystems.targeting;

import java.lang.annotation.Target;
import java.util.List;

import org.wildstang.framework.core.Core;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionConsts {

    public static final double inToM = 1/39.71;
    public final double mToIn = 39.71;

    // All Variables in Blue coordinate system
    public static final List<Pose2d> leftTopBranches = List.of();
    public static final List<Pose2d> rightTopBranches = List.of();
    public static final List<Pose2d> leftBottomBranches = List.of();
    public static final List<Pose2d> rightBottomBranches = List.of();
    public static final Translation2d reefCenter = new Translation2d(176, 158.5);
    public static final double halfwayAcrossFieldY = 4.0;
    // TODO: what are the actual values?
    public static final double coralStationLeftHeading = Double.NaN;
    public static final double coralStationRightHeading = Double.NaN;

    // X value of the translation is irrelevant
    public static final Translation2d netScore = new Translation2d();

    //TODO: what are the actual pipeline indices?
    public static final int ATPipelineIndex = 0;

    /*
     * April Tag IDs:
     * - Stage: 11, 12, 13, 14, 15, 16
     * - Speaker: 3, 4, 7, 8
     * - Amp: 5, 6
     */

    public static Pose3d cameraPose = new Pose3d(-11.8*inToM,8.5*inToM,22.3*inToM, new Rotation3d(0.0,-65*Math.PI/180.0,11*Math.PI/180.0));

}