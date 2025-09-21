package org.wildstang.sample.auto.Programs;


import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.ObjectIntakeStep;
import org.wildstang.sample.auto.Steps.RunGroundStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.auto.Steps.SwerveMultiPointStep;
import org.wildstang.sample.auto.Steps.SwerveToPointStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.VisionConsts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BlueLeftFiveCoral extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.15, 5.48, 0, Alliance.Blue));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 1st Coral
        addStep(new SwerveToPointStep(swerve,  (VisionConsts.rightBranchBackLeft)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new RunGroundStep());
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));

        // Pickup 2nd Coral
        // Moves away from the reef and then drives to a point to see ground coral
        addStep(new SwerveMultiPointStep(new Pose2d[] {new Pose2d(4.3, 8.8, Rotation2d.fromDegrees(90)), new Pose2d(3.61, 5.59, Rotation2d.fromDegrees(133))}, new double[] {}, 1.5));
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new ObjectIntakeStep());
        group1.addStep(new IntakeCoralStep());
        addStep(group1);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Scores 2nd Coral
        addStep(new SwerveToPointStep(swerve,  (VisionConsts.leftBranchFrontLeft)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));

        // Pickup 3rd Coral
        addStep(new SwerveToPointStep(swerve, new Pose2d(3.61, 5.59, Rotation2d.fromDegrees(133))));
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new ObjectIntakeStep());
        group2.addStep(new IntakeCoralStep());
        addStep(group2);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 3rd Coral
        addStep(new SwerveToPointStep(swerve,  (VisionConsts.rightBranchFrontLeft)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        
        // Pickup 4th Coral
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new ObjectIntakeStep());
        group3.addStep(new IntakeCoralStep());

        // Score 4th Coral
        addStep(new SwerveToPointStep(swerve,  (VisionConsts.leftBranchFront)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));

        //  Pickup 5th Coral :)
        AutoParallelStepGroup group4 = new AutoParallelStepGroup();
        group4.addStep(new SwerveMultiPointStep(new Pose2d[] {new Pose2d(2, 4.04, Rotation2d.fromDegrees(180)), new Pose2d(1.38, 4.04, Rotation2d.fromDegrees(133))}, new double[] {}));
        group4.addStep(new IntakeCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 5th Coral
        addStep(new SwerveToPointStep(swerve, VisionConsts.rightBranchFront));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());

        // Our Job is done
    }

    @Override
    public String toString() {
        return "(V2) Blue Left Five Coral";
    }
}
