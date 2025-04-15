package org.wildstang.sample.auto.Programs;


import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AlgaePickStep;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.GroundIntakeCoralStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.ObjectIntakeStep;
import org.wildstang.sample.auto.Steps.RunGroundStep;
import org.wildstang.sample.auto.Steps.ScoreCoralStep;
import org.wildstang.sample.auto.Steps.SuperStructureSmartStep;
import org.wildstang.sample.auto.Steps.SwerveMultiPointStep;
import org.wildstang.sample.auto.Steps.SwerveToObjectStep;
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

public class RedLeftFourObject extends AutoProgram {

    @Override
   protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.15, 5.48, 0, Alliance.Red));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        addStep(new RunGroundStep(false));

        // Score 1st Coral
        AutoParallelStepGroup score1 = new AutoParallelStepGroup();
        score1.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchBackLeft)));
        score1.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        addStep(score1);
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));

        // Pickup 2nd Coral
        // Moves away from the reef and then drives to a point to see ground coral
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        AutoSerialStepGroup group1a = new AutoSerialStepGroup();
        AutoSerialStepGroup group1b = new AutoSerialStepGroup();
        group1a.addStep(new SwerveToObjectStep(swerve, new Pose2d(new Translation2d(3.2, 6.75), Rotation2d.fromDegrees(-200)), 1.0));
        group1a.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFrontLeft)));
        group1b.addStep(new GroundIntakeCoralStep());
        group1b.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        group1.addStep(group1a);
        group1.addStep(group1b);
        addStep(group1);
        
        // Scores 2nd Coral
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));

        // Pickup 3rd Coral
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        AutoSerialStepGroup group2a = new AutoSerialStepGroup();
        AutoSerialStepGroup group2b = new AutoSerialStepGroup();
        group2a.addStep(new SwerveToObjectStep(swerve, new Pose2d(new Translation2d(3.0, 6.0), Rotation2d.fromDegrees(135)), 0));
        group2a.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.rightBranchFrontLeft)));
        group2b.addStep(new GroundIntakeCoralStep());
        group2b.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        group2.addStep(group2a);
        group2.addStep(group2b);
        addStep(group2);

        // Score 3rd Coral
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.GROUND_INTAKE));
        
        // Pickup 4th Coral
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        AutoSerialStepGroup group3a = new AutoSerialStepGroup();
        AutoSerialStepGroup group3b = new AutoSerialStepGroup();
        group3a.addStep(new SwerveToObjectStep(swerve, new Pose2d(new Translation2d(3.0, 6.0), Rotation2d.fromDegrees(135)), 0));
        // group3a.addStep(new SwerveMultiPointStep(new Pose2d[] {new Pose2d(4.5, 5.75, Rotation2d.fromDegrees(-300)), 
        group3a.addStep(new SwerveToPointStep(swerve, VisionConsts.flipRot(VisionConsts.leftBranchFrontLeft)));
        //     VisionConsts.flipRot(VisionConsts.rightBranchBackLeft)}, new double[] {}, 0));
        group3b.addStep(new GroundIntakeCoralStep());
        group3b.addStep(new AlgaePickStep(SuperstructurePosition.ALGAE_PREPICK_LOW));
        //group3b.addStep(new SuperStructureSmartStep(SuperstructurePosition.CORAL_REEF_L4));
        group3.addStep(group3a);
        group3.addStep(group3b);
        addStep(group3);

        // Score 4th Coral
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L3));
        addStep(new ScoreCoralStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.AUTO_AVOID));
        addStep(new AutoStepDelay(500));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        //If we need to add more to this I'll be a very happy man
        // Our Job is done
    }

    @Override
    public String toString() {
        return "Red Left Four Object";
    }
}
