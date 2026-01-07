package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

public class SuperStructureSmartStep extends AutoStep{

    private SuperstructureSubsystem superstructure;
    private SwerveDrive swerve;
    private SuperstructurePosition pos;
    private double dist;

    public SuperStructureSmartStep(SuperstructurePosition newpos){
        pos = newpos;
        dist = 0.75;
    }
    public SuperStructureSmartStep(SuperstructurePosition newpos, double distance){
        pos = newpos;
        dist = distance;
    }

    @Override
    public void initialize() {
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void update() {
        if (swerve.distanceToTarget() < dist){
            superstructure.setPosition(pos);
            setFinished();
        } else superstructure.setPosition(SuperstructurePosition.STOWED_UP);
    }

    @Override
    public String toString() {
        return "Superstructure Smart Step";
    }
    
}
