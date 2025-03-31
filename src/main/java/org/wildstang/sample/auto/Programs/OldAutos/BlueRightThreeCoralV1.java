package org.wildstang.sample.auto.Programs.OldAutos;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.auto.Steps.SwerveAutoStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BlueRightThreeCoralV1 extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.15, 2.57, 0, Alliance.Blue));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 1st Coral
        addStep(new SwervePathFollowerStep("BlueRightThreeCoralV1", swerve, 0));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));
        addStep(new AutoStepDelay(100));

        // Pickup 2nd Coral
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SwerveAutoStep("BlueRightThreeCoralV1", swerve, 1));
        AutoSerialStepGroup group1a = new AutoSerialStepGroup();
        group1a.addStep(new AutoStepDelay(500));
        group1a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));
        group1a.addStep(new IntakeCoralStep());
        group1.addStep(group1a);
        addStep(group1);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 2nd Coral
        addStep(new SwervePathFollowerStep("BlueRightThreeCoralV1", swerve, 2));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));
        addStep(new AutoStepDelay(100));

        // Pickup 3rd Coral
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        group2.addStep(new SwerveAutoStep("BlueRightThreeCoralV1", swerve, 3));
        AutoSerialStepGroup group2a = new AutoSerialStepGroup();
        group2a.addStep(new AutoStepDelay(500));
        group2a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));
        group2a.addStep(new IntakeCoralStep());
        group2.addStep(group2a);
        addStep(group2);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));

        // Score 3rd Coral
        addStep(new SwervePathFollowerStep("BlueRightThreeCoralV1", swerve, 4));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));
    }

    @Override
    public String toString() {
        return "Blue Right Three Coral V1";
    }
    
}
