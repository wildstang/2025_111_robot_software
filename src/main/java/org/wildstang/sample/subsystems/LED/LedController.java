package org.wildstang.sample.subsystems.LED;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.targeting.WsPose;
import org.wildstang.sample.subsystems.LED.Blinkin.BlinkinValues;


public class LedController implements Subsystem {

    private WsPose pose;
    private Blinkin led;
    private BlinkinValues color;

    @Override
    public void update(){
        if (Core.getIsDisabledMode()){
            if (Core.isAutoLocked()){
                if (Core.isBlue()) color = BlinkinValues.TWINKLES_OCEAN_PALETTE;
                else color = BlinkinValues.TWINKLES_LAVA_PALETTE;
            } else color = BlinkinValues.RAINBOW_RAINBOW_PALETTE;
        } else {

            
            if (Core.isAutoLocked()){
                if (Core.isBlue()) color = BlinkinValues.TWINKLES_OCEAN_PALETTE;
                else color = BlinkinValues.TWINKLES_LAVA_PALETTE;
            } else color = BlinkinValues.RAINBOW_RAINBOW_PALETTE;
        }
        led.setColor(color);
    }

    @Override
    public void inputUpdate(Input source) {
        
    }

    @Override
    public void initSubsystems() {      
        pose = (WsPose) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_POSE);   
    }

    @Override
    public void init() {
        
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
