package org.wildstang.sample.auto.Programs.OldAutos;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.ObjectIntakeStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.auto.Steps.SuperStructureSmartStep;
import org.wildstang.sample.auto.Steps.SwerveAutoStep;
import org.wildstang.sample.auto.Steps.SwerveToPointStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.VisionConsts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BlueRightFourFast extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.15, 2.57, 0, Alliance.Blue));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 1st Coral
        AutoParallelStepGroup move1 = new AutoParallelStepGroup();
        move1.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchBackRight)));
        move1.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(move1);
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));

        // Pickup and move to score 2nd Coral
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        AutoSerialStepGroup group1b = new AutoSerialStepGroup();
        group1b.addStep(new SwerveToPointStep(swerve,new Pose2d(1.32, 0.95, Rotation2d.fromDegrees(-125))));
        group1b.addStep(new AutoStepDelay(50));
        group1b.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.rightBranchFrontRight)));
        AutoSerialStepGroup group1a = new AutoSerialStepGroup();
        group1a.addStep(new AutoStepDelay(500));
        group1a.addStep(new IntakeCoralStep());
        group1a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        group1a.addStep(new AutoStepDelay(100));
        group1a.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        group1.addStep(group1a);
        group1.addStep(group1b);
        addStep(group1);

        // Score 2nd Coral
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));

        // Pickup and move to score 3rd Coral
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        AutoSerialStepGroup group2b = new AutoSerialStepGroup();
        group2b.addStep(new SwerveToPointStep(swerve,new Pose2d(1.32, 0.95, Rotation2d.fromDegrees(-125))));
        group2b.addStep(new AutoStepDelay(50));
        group2b.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFrontRight)));
        AutoSerialStepGroup group2a = new AutoSerialStepGroup();
        group2a.addStep(new AutoStepDelay(500));
        group2a.addStep(new IntakeCoralStep());
        group2a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        group2a.addStep(new AutoStepDelay(100));
        group2a.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        group2.addStep(group2a);
        group2.addStep(group2b);
        addStep(group2);

        // Score 3rd Coral
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        // Pickup and move to score 4th Coral
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        AutoSerialStepGroup group3b = new AutoSerialStepGroup();
        group3b.addStep(new SwerveToPointStep(swerve,new Pose2d(1.32, 0.95, Rotation2d.fromDegrees(-125))));
        group3b.addStep(new AutoStepDelay(50));
        group3b.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.rightBranchBackRight)));
        AutoSerialStepGroup group3a = new AutoSerialStepGroup();
        group3a.addStep(new AutoStepDelay(500));
        group3a.addStep(new IntakeCoralStep());
        group3a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        group3a.addStep(new AutoStepDelay(100));
        group3a.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        group3.addStep(group3a);
        group3.addStep(group3b);
        addStep(group3);

        // Score 4th Coral
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));
    }

    @Override
    public String toString() {
        return "Blue Right Four Fast";
    }
}
