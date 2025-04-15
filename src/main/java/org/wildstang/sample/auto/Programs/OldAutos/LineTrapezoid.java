package org.wildstang.sample.auto.Programs.OldAutos;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.SwervePathFollowerStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.SwerveTrapezoidalStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// Test swerve auto program, named per Arms
public class LineTrapezoid extends AutoProgram {

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(2, 6.5, 0, Alliance.Blue));
        addStep(new SwerveTrapezoidalStep(swerve, new Pose2d(6.0, 6.5, new Rotation2d(Math.toRadians(90)))));
    }

    @Override
    public String toString() {
        return "Line Trapezoid";
    }
}
