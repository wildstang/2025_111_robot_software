package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.GroundIntake;

import edu.wpi.first.wpilibj.Timer;

public class GroundIntakeCoralStep extends AutoStep {
    CoralPath coralPath;   
    GroundIntake ground;
    double timeout;
    Timer timer = new Timer();

    /**
     * Intake Coral Step
     * Sets the coral path to intake and runs until coral is detected or timeout elapsed, whichever occurs first
     * @param timeout Amount of time to wait before ending step
     */
    public GroundIntakeCoralStep(double timeout) {
        this.timeout = timeout;
    }

    /** 
     * Intake Coral Step
     * Sets the coral path to intake and runs until coral is detected
     */
    public GroundIntakeCoralStep() {
        this.timeout = 3;
    }

    @Override
    public void initialize() {
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        ground = (GroundIntake) Core.getSubsystemManager().getSubsystem(WsSubsystems.GROUND_INTAKE);
        timer.start();
        coralPath.setIntake(CoralPath.IntakeState.INTAKING);
        ground.groundOn();
    }

    @Override
    public void update() {
        if (coralPath.hasCoral()) {
            coralPath.setIntake(CoralPath.IntakeState.NEUTRAL);
            ground.groundOff();
            this.setFinished();
        } else if (timer.hasElapsed(timeout)) {
            ground.groundL1();
            if (timer.hasElapsed(timeout + 0.25)){
                ground.groundOn();
                 timer.reset();
            }
        } 
     }

    @Override
    public String toString() {
        return "Ground Intake Coral Step";
    }
    
}
