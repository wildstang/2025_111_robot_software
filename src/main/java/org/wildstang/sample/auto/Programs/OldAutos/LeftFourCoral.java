package org.wildstang.sample.auto.Programs.OldAutos;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.RunGroundStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.auto.Steps.SwerveToPointStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.VisionConsts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LeftFourCoral extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        // TODO: Parallelize auto setup step so we don't have to wait the cycles for pigeon to update over CAN
        addStep(new AutoSetupStep(7.15, 5.48, 0, Alliance.Blue));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 1st Coral
        addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.rightBranchBackLeft)));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));

        // Pickup 2nd Coral
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwerveToPointStep(swerve,new Pose2d(1.54, 7.34, Rotation2d.fromDegrees(-235))));
        AutoSerialStepGroup group1a = new AutoSerialStepGroup();
        group1a.addStep(new AutoStepDelay(500));
        group1a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));
        group1a.addStep(new IntakeCoralStep());
        group1.addStep(group1a);
        addStep(group1);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 2nd Coral
        addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.rightBranchFrontLeft)));
        addStep(new RunGroundStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        // Pickup 3rd Coral
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        AutoSerialStepGroup group2b = new AutoSerialStepGroup();
        group2b.addStep(new SwerveToPointStep(swerve, new Pose2d(2.0, 5.92, Rotation2d.fromRadians(Math.PI))));
        group2b.addStep(new SwerveToPointStep(swerve, new Pose2d(1.38, 5.92, Rotation2d.fromRadians(Math.PI))));
        group2b.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        AutoSerialStepGroup group2a = new AutoSerialStepGroup();
        group2a.addStep(new AutoStepDelay(500));
        group2a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        group2a.addStep(new IntakeCoralStep());
        group2a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        group2.addStep(group2a);
        group2.addStep(group2b);
        addStep(group2);

        // Score 3rd Coral
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        // Pickup 4th Coral!!!!
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        AutoSerialStepGroup group3b = new AutoSerialStepGroup();
        group3b.addStep(new SwerveToPointStep(swerve, new Pose2d(2.0, 4.04, Rotation2d.fromRadians(Math.PI))));
        group3b.addStep(new SwerveToPointStep(swerve, new Pose2d(1.38, 4.04, Rotation2d.fromRadians(Math.PI))));
        group3b.addStep(new SwerveToPointStep(swerve, VisionConsts.rightBranchFront));
        AutoSerialStepGroup group3a = new AutoSerialStepGroup();
        group3a.addStep(new AutoStepDelay(500));
        group3a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        group3a.addStep(new IntakeCoralStep());
        group3a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        group3.addStep(group3a);
        group3.addStep(group3b);
        addStep(group3);

        // Score 4th Coral
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));
    }

    @Override
    public String toString() {
        return "Blue Left Four Coral V1";
    }
    
}
