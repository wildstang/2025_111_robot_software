package org.wildstang.sample.auto.Programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoParallelStepGroup;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.auto.steps.control.AutoStepDelay;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.auto.Steps.AlgaePickStep;
import org.wildstang.sample.auto.Steps.AutoSetupStep;
import org.wildstang.sample.auto.Steps.GroundIntakeCoralStep;
import org.wildstang.sample.auto.Steps.GroundStationStep;
import org.wildstang.sample.auto.Steps.IntakeCoralStep;
import org.wildstang.sample.auto.Steps.RunGroundStep;
import org.wildstang.sample.auto.Steps.ScoreAlgaeFrontStep;
import org.wildstang.sample.auto.Steps.ScoreAlgaeStep;
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

public class RedCenterThree extends AutoProgram{

    @Override
    protected void defineSteps() {
        SwerveDrive swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        addStep(new AutoSetupStep(7.15, 3.93, 0, Alliance.Red));
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));

        //intake down, arm to algae low, pick sequence, move to back left
        AutoParallelStepGroup group1 = new AutoParallelStepGroup();
        group1.addStep(new RunGroundStep(false));
        group1.addStep(new AlgaePickStep(SuperstructurePosition.ALGAE_PREPICK_LOW));
        group1.addStep(new SwerveToPointStep(swerve,  (VisionConsts.leftBranchBack)));
        addStep(group1);

        //score l4 pos, then score coral
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.CORAL_REEF_L3));
        addStep(new ScoreCoralStep());

        //superstructure to stow up, move t onet, smart step to net score
        AutoParallelStepGroup group2 = new AutoParallelStepGroup();
        AutoSerialStepGroup group2a = new AutoSerialStepGroup();
        group2a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        group2a.addStep(new AutoStepDelay(1500));
        group2a.addStep(new SuperStructureSmartStep(SuperstructurePosition.ALGAE_NET_THROW_AUTO));
        group2.addStep(new SwerveToPointStep(swerve, new Pose2d(new Translation2d(VisionConsts.netScore.getX(), 5.0), 
            Rotation2d.fromDegrees(0))));
        group2.addStep(group2a);
        addStep(group2);

        //score algae
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.ALGAE_NET_THROW_FRONT));
        addStep(new ScoreAlgaeStep());

        //superstructure to algae high, move to back left reef, pick sequence
        AutoParallelStepGroup group3 = new AutoParallelStepGroup();
        group3.addStep(new AlgaePickStep(SuperstructurePosition.ALGAE_PREPICK_HIGH));
        group3.addStep(new SwerveToPointStep(swerve,  (VisionConsts.leftBranchBackLeft)));
        addStep(group3);

        //superstructure to stow up, move to net, smart step to net score
        AutoParallelStepGroup group4 = new AutoParallelStepGroup();
        AutoSerialStepGroup group4a = new AutoSerialStepGroup();
        group4a.addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED_UP));
        group4a.addStep(new AutoStepDelay(1000));
        group4a.addStep(new SuperStructureSmartStep(SuperstructurePosition.ALGAE_NET_BACK));
        group4.addStep(new SwerveToPointStep(swerve, new Pose2d(new Translation2d(VisionConsts.netScore.getX(), 6.0), 
            Rotation2d.fromDegrees(180))));
        group4.addStep(group4a);
        addStep(group4);

        //score algae
        //addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.ALGAE_NET_THROW_FRONT));
        addStep(new ScoreAlgaeFrontStep());
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));
        //addStep(new SwerveToPointStep(swerve, new Pose2d(4.54, 7.34, Rotation2d.fromDegrees(180))));

        //2 serial group: object intake and move to back left
        AutoParallelStepGroup group5 = new AutoParallelStepGroup();
        AutoSerialStepGroup group5a = new AutoSerialStepGroup();
        AutoSerialStepGroup group5b = new AutoSerialStepGroup();

        //grab third ball and bring it close to the net
        group5a.addStep(new SwerveMultiPointStep(new Pose2d[]{ 
                new Pose2d(new Translation2d(6.5, 3.0), Rotation2d.fromDegrees(-90)),
                 (VisionConsts.leftBranchBackRight)}, new double[]{}));
        group5b.addStep(new AutoStepDelay(1000));
        group5b.addStep(new AlgaePickStep(SuperstructurePosition.ALGAE_PREPICK_HIGH));
        group5.addStep(group5a);
        group5.addStep(group5b);
        addStep(group5);
        addStep(SuperstructureSubsystem.setPositionStep(SuperstructurePosition.STOWED));
        addStep(new SwerveMultiPointStep(new Pose2d[]{
                new Pose2d(new Translation2d(6.5, 3.0), Rotation2d.fromDegrees(-90)),
                new Pose2d(new Translation2d(6.5, 6.0), Rotation2d.fromDegrees(180))}, new double[]{}));
        //end grabbing third ball
    }

    @Override
    public String toString() {
        return "Red Center Three";
    }
    
}
