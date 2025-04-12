package org.wildstang.sample.auto.Steps;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.Superstructure.SuperstructurePosition;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class ScoreAlgaeStep extends AutoStep{

    CoralPath coralPath;
    SuperstructureSubsystem superstructure;
    Timer scoreTimer = new Timer();

    @Override
    public void initialize() {
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
    }

    @Override
    public void update() {
        if (superstructure.isAtPosition()){
            superstructure.setPosition(SuperstructurePosition.ALGAE_NET_THROW_FRONT);
            coralPath.scoreAlgae();
            scoreTimer.start();
        }
        if (scoreTimer.hasElapsed(0.5)){
            coralPath.stopAlgae();
            setFinished();
        }
        
    }

    @Override
    public String toString() {
        return "Score Algae Step";
    }
    
}
