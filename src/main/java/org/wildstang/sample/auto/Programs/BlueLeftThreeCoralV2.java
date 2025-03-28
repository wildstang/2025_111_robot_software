package org.wildstang.sample.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.ObjectIntakeStep;
import org.wildstang.sample.auto.Steps.RunGroundStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.auto.Steps.SwerveAutoStep;
import org.wildstang.sample.auto.Steps.SwerveMultiPointStep;
import org.wildstang.sample.auto.Steps.SwerveToPointStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.VisionConsts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BlueLeftThreeCoralV2 extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.15, 5.48, 0, Alliance.Blue));
        //tush push
        //addStep(new SwerveToPointStep(swerve, new Pose2d(new Translation2d(7.45, 5.48), Rotation2d.fromDegrees(0))));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 1st Coral
        addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchBackLeft)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));

        // Pickup 2nd Coral
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwerveToPointStep(swerve,new Pose2d(1.54, 7.34, Rotation2d.fromDegrees(-235))));
        AutoSerialStepGroup group1a = new AutoSerialStepGroup();
        group1a.addStep(new AutoStepDelay(500));
        group1a.addStep(new IntakeCoralStep());
        group1.addStep(group1a);
        addStep(group1);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 2nd Coral
        addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.rightBranchFrontLeft)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));

        // Pickup 3rd Coral
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwerveToPointStep(swerve,new Pose2d(1.54, 7.34, Rotation2d.fromDegrees(-235))));
        AutoSerialStepGroup group2a = new AutoSerialStepGroup();
        group2a.addStep(new AutoStepDelay(500));
        group2a.addStep(new IntakeCoralStep());
        group2.addStep(group2a);
        addStep(group2);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 3rd Coral
        addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFrontLeft)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new RunGroundStep());
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        //grab 4th coral
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        group3.addStep(new ObjectIntakeStep());
        group3.addStep(new IntakeCoralStep());
        addStep(group3);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        addStep(new SwerveMultiPointStep(new Pose2d[] {new Pose2d(4.5, 5.75, Rotation2d.fromDegrees(-300)), 
            VisionConsts.flipRot(VisionConsts.rightBranchBackLeft)}, new double[] {}, 0));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
    }

    @Override
    public String toString() {
        return "Blue Left Three V2";
    }
}
