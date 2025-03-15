package org.wildstang.sample.subsystems.LED;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.LED.Blinkin.BlinkinValues;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;


public class LedController implements Subsystem {

    private SuperstructureSubsystem superstructure;
    private SwerveDrive swerve;
    private CoralPath coralPath;
    private Blinkin led;
    private BlinkinValues color;
    private WsJoystickAxis driver_LT;
    private boolean isScoring = false;

    @Override
    public void update(){
        if (isScoring && superstructure.isAtPosition()){
            color = BlinkinValues.GREEN;
        } else if (isScoring && !superstructure.isAtPosition()){
            color = BlinkinValues.DARK_RED;
        } else if (swerve.isScoringAlgae()){
            color = BlinkinValues.BLUE;
        } else if (superstructure.isAlgaeRemoval()){
            color = BlinkinValues.WHITE;
        } else color = BlinkinValues.RAINBOW_RAINBOW_PALETTE;
        led.setColor(color);
    }

    @Override
    public void inputUpdate(Input source) {
        isScoring = Math.abs(driver_LT.getValue()) > 0.5;
    }

    @Override
    public void initSubsystems() {      
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    @Override
    public void init() {
        driver_LT = (WsJoystickAxis) WsInputs.DRIVER_LEFT_TRIGGER.get();
        driver_LT.addInputListener(this);
        
        //Outputs
        led = new Blinkin(0);
        color = BlinkinValues.RAINBOW_RAINBOW_PALETTE;
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return "Led Controller";
    }
}
