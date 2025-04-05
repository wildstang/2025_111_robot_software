package org.wildstang.sample.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.ObjectIntakeStep;
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
        addStep(new SwerveMultiPointStep(new Pose2d[] {new Pose2d(4, 6.6, Rotation2d.fromRadians(Math.PI)), 
            new Pose2d(2.8, 5.5, Rotation2d.fromRadians(Math.PI)), 
            VisionConsts.flipRot(VisionConsts.rightBranchFront)}, new double[] {}));
        //AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        //group1.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        //group1.addStep(new SwerveTrapezoidalStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        //group1.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        //addStep(group1);    
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L4));    
        addStep(new ScoreCoralStep());
        addStep(new RunGroundStep());        
        
        // Pickup 2nd Coral
        AutoParallelStepGroup getCoral1 = new AutoParallelStepGroup();
        AutoSerialStepGroup drive1 = new AutoSerialStepGroup();
        //drive1.addStep(new SwerveToPointStep(swerve, new Pose2d(1.25, 3.8, Rotation2d.fromRadians(Math.PI))));
        drive1.addStep(new ObjectIntakeStep());
        drive1.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        AutoSerialStepGroup intake1 = new AutoSerialStepGroup();
        //intake1.addStep(new AutoStepDelay(200));
        intake1.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        intake1.addStep(new IntakeCoralStep(3.0));
        intake1.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        getCoral1.addStep(intake1);
        getCoral1.addStep(drive1);
        addStep(getCoral1);

        addStep(new ScoreCoralStep());
        
        // Pickup 3rd Coral
        AutoParallelStepGroup getCoral2 = new AutoParallelStepGroup();
        AutoSerialStepGroup drive2 = new AutoSerialStepGroup();
        drive2.addStep(new SwerveToPointStep(swerve, new Pose2d(1.44, 5.84, Rotation2d.fromRadians(3.0*Math.PI/4.0))));
        drive2.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFront)));
        AutoSerialStepGroup intake2 = new AutoSerialStepGroup();
        intake2.addStep(new AutoStepDelay(200));
        intake2.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        intake2.addStep(new IntakeCoralStep(3.0));
        intake2.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L2));
        getCoral2.addStep(intake2);
        getCoral2.addStep(drive2);
        addStep(getCoral2);

        addStep(new ScoreCoralStep(0.0));

        // Pickup 4th Coral
        AutoParallelStepGroup getCoral3 = new AutoParallelStepGroup();
        AutoSerialStepGroup drive3 = new AutoSerialStepGroup();
        drive3.addStep(new SwerveToPointStep(swerve, new Pose2d(1.33, 2.5, Rotation2d.fromRadians(5.0*Math.PI/4.0))));
        drive3.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.rightBranchFront)));
        AutoSerialStepGroup intake3 = new AutoSerialStepGroup();
        intake2.addStep(new AutoStepDelay(200));
        intake3.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        intake3.addStep(new IntakeCoralStep());
        intake3.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L2));
        getCoral3.addStep(intake3);
        getCoral3.addStep(drive3);
        addStep(getCoral3);

        addStep(new ScoreCoralStep(0.0));
    }

    @Override
    public String toString() {
        return "Blue Left Jack In The Bot auto";
    }
    
}
