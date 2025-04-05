package org.wildstang.sample.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.RunGroundStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.auto.Steps.SuperStructureSmartStep;
import org.wildstang.sample.auto.Steps.SwerveMultiPointStep;
import org.wildstang.sample.auto.Steps.SwerveToPointStep;
import org.wildstang.sample.auto.Steps.SwerveTrapezoidalStep;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.VisionConsts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BlueLeftJackInTheBot extends AutoProgram {

    @Override
    protected void defineSteps() {
        
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.15, 6.9, 180, Alliance.Blue));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));


        // Score 1st Coral
        addStep(new SwerveMultiPointStep(new Pose2d[] {new Pose2d(4, 5.8, Rotation2d.fromRadians(Math.PI)), 
            new Pose2d(2.6, 4.14, Rotation2d.fromRadians(Math.PI))}, new double[] {}));
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        group1.addStep(new SwerveTrapezoidalStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        addStep(group1);        
        addStep(new ScoreCoralStep());
        addStep(new RunGroundStep());        
        
        // Pickup 2nd Coral
        AutoParallelStepGroup getCoral1 = new AutoParallelStepGroup();
        AutoSerialStepGroup drive1 = new AutoSerialStepGroup();
        drive1.addStep(new SwerveToPointStep(swerve, new Pose2d(1.38, 4.04, Rotation2d.fromRadians(Math.PI))));
        drive1.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        AutoSerialStepGroup intake1 = new AutoSerialStepGroup();
        intake1.addStep(new AutoStepDelay(500));
        intake1.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        intake1.addStep(new IntakeCoralStep());
        intake1.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        getCoral1.addStep(intake1);
        getCoral1.addStep(drive1);

        addStep(new ScoreCoralStep());
        
        // Pickup 3rd Coral
        AutoParallelStepGroup getCoral2 = new AutoParallelStepGroup();
        AutoSerialStepGroup drive2 = new AutoSerialStepGroup();
        drive1.addStep(new SwerveToPointStep(swerve, new Pose2d(1.38, 4.04, Rotation2d.fromRadians(Math.PI))));
        drive1.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        AutoSerialStepGroup intake2 = new AutoSerialStepGroup();
        intake1.addStep(new AutoStepDelay(500));
        intake1.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        intake1.addStep(new IntakeCoralStep());
        intake1.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        getCoral1.addStep(intake1);
        getCoral1.addStep(drive1);
    }

    @Override
    public String toString() {
        return "Blue Left Jack In The Bot auto";
    }
    
}
