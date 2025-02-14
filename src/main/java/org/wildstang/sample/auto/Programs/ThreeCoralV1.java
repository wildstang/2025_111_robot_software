package org.wildstang.sample.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ThreeCoralV1 extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.58, 2.43, 180, Alliance.Blue));

        // Score 1st Coral
        addStep(new SwervePathFollowerStep("ThreeCoralPath", swerve, 0));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        // Pickup 2nd Coral
        addStep(new SwervePathFollowerStep("ThreeCoralPath", swerve, 1));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));
        addStep(new IntakeCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        // Score 2nd Coral
        addStep(new SwervePathFollowerStep("ThreeCoralPath", swerve, 2));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        // Pickup 3rd Coral
        addStep(new SwervePathFollowerStep("ThreeCoralPath", swerve, 3));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_STATION_FRONT));
        addStep(new IntakeCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        // Score 3rd Coral
        addStep(new SwervePathFollowerStep("ThreeCoralPath", swerve, 4));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(new ScoreCoralStep());
    }

    @Override
    public String toString() {
        // TODO Auto-generated method stub
        return "Three Coral V1";
    }
    
}
