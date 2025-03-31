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
    public final double SCORE_DURATION = 0.4;
    public final double AT_POS_DURATION = 0.25;
    private boolean hasStarted = false;
    CoralPath coralPath;
    SuperstructureSubsystem superstructure;
    SwerveDrive swerveDrive;
    Timer scoreTimer = new Timer();
    Timer atPosTimer = new Timer();

    @Override
    public void initialize() {
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
        swerveDrive = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void update() {
        if (!hasStarted){
            hasStarted = true;
            if (!coralPath.hasCoral()) setFinished();
        }

        // Execute once
        if (superstructure.isAtPosition() && swerveDrive.isAtPosition() && atPosTimer.isRunning() == false) {
            atPosTimer.start();
        }
        // Execute once
        if (atPosTimer.hasElapsed(AT_POS_DURATION) && scoreTimer.isRunning() == false) {
            coralPath.setIntake(CoralPath.IntakeState.SCORING);
            scoreTimer.start();
        }
        if (scoreTimer.hasElapsed(SCORE_DURATION)) {
            coralPath.setIntake(CoralPath.IntakeState.NEUTRAL);
            setFinished();
        }
     }

    @Override
    public String toString() {
        return "Score Coral Step";
    }
    
}
