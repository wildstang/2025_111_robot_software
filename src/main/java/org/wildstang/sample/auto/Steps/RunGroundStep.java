package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.GroundIntake;

public class RunGroundStep extends AutoStep {

    GroundIntake groundIntake;

    @Override
    public void initialize() {
        groundIntake = (GroundIntake) Core.getSubsystemManager().getSubsystem(WsSubsystems.GROUND_INTAKE);
    }

    @Override
    public void update() {
        groundIntake.deploy();
        groundIntake.groundOn();
    }

    @Override
    public String toString() {
        return "Run Ground Intake Step";
    }
    
}
