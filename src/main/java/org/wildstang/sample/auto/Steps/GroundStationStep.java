package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.GroundIntake;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;

public class GroundStationStep extends AutoStep {

    GroundIntake groundIntake;

    @Override
    public void initialize() {
        groundIntake = (GroundIntake) Core.getSubsystemManager().getSubsystem(WsSubsystems.GROUND_INTAKE);
    }

    @Override
    public void update() {
        groundIntake.stationPickup();
        this.setFinished();
    }

    @Override
    public String toString() {
        return "Ground Station Step";
    }
    
}
