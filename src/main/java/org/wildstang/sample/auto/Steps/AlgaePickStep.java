package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;

public class AlgaePickStep extends AutoStep{

    private SuperstructureSubsystem superstructure;
    private CoralPath coralPath;
    private SuperstructurePosition pos;

    public AlgaePickStep(SuperstructurePosition newpos){
        pos = newpos;
    }

    @Override
    public void initialize() {
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
    }

    @Override
    public void update() {
        superstructure.setPosition(pos);
        coralPath.getAlgae();
        if (coralPath.hasAlgae()){
            setFinished();
        }
        
    }

    @Override
    public String toString() {
        return "Algae Pick Step";
    }
    
}
