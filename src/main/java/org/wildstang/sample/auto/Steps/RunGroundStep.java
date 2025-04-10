package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.GroundIntake;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;

public class RunGroundStep extends AutoStep {

    GroundIntake groundIntake;
    private boolean runRollers = false;

    public RunGroundStep(){}
    public RunGroundStep(boolean enable){
        runRollers = enable;
    }

    @Override
    public void initialize() {
        groundIntake = (GroundIntake) Core.getSubsystemManager().getSubsystem(WsSubsystems.GROUND_INTAKE);
    }

    @Override
    public void update() {
        groundIntake.deploy();
        if (runRollers) groundIntake.groundOn();
        this.setFinished();
    }

    @Override
    public String toString() {
        return "Run Ground Intake Step";
    }
    
}
