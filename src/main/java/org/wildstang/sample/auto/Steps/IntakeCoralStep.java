package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;

import edu.wpi.first.wpilibj.Timer;

public class IntakeCoralStep extends AutoStep {
    CoralPath coralPath;   
    double timeout;
    Timer timer = new Timer();

    /**
     * Intake Coral Step
     * Sets the coral path to intake and runs until coral is detected or timeout elapsed, whichever occurs first
     * @param timeout Amount of time to wait before ending step
     */
    public IntakeCoralStep(double timeout) {
        this.timeout = timeout;
    }

    /** 
     * Intake Coral Step
     * Sets the coral path to intake and runs until coral is detected
     */
    public IntakeCoralStep() {
        this.timeout = Double.MAX_VALUE;
    }

    @Override
    public void initialize() {
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        timer.start();
        coralPath.setIntake(CoralPath.IntakeState.INTAKING);
    }

    @Override
    public void update() {
        if (coralPath.hasCoral()) {
            coralPath.setIntake(CoralPath.IntakeState.NEUTRAL);
            this.setFinished();
        } else if (timer.hasElapsed(timeout)) {
            coralPath.setIntake(CoralPath.IntakeState.NEUTRAL);
            this.setFinished();
        }
     }

    @Override
    public String toString() {
        return "Intake Coral Step";
    }
    
}
