package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.Timer;

/**
 * Runs the coral path for SCORE_DURATION seconds once the superstructure and swerve drive are at position
 * Intended to be used after calling setSuperstructurePositionStep
 */
public class ScoreCoralStep extends AutoStep {
    public static final double SCORE_DURATION = 0.2;
    CoralPath coralPath;
    SuperstructureSubsystem superstructure;
    SwerveDrive swerveDrive;
    Timer timer = new Timer();

    @Override
    public void initialize() {
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
        swerveDrive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void update() {

        // Execute once
        if (superstructure.isAtPosition() && swerveDrive.isAtPosition() && timer.isRunning() == false) {
            coralPath.setScore(true);
            timer.start();
        }
        if (timer.hasElapsed(SCORE_DURATION)) {
            coralPath.setScore(false);
        }
     }

    @Override
    public String toString() {
        return "Score Coral Step";
    }
    
}
