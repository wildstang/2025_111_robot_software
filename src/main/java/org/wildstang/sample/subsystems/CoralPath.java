package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsAnalogInput;
import org.wildstang.hardware.roborio.inputs.WsDigitalInput;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
//import au.grapplerobotics.LaserCan;

import edu.wpi.first.wpilibj.Timer;

public class CoralPath implements Subsystem{

    private static final double CORAL_CURRENT_LIMIT = 30;
    private static final double ALGAE_CURRENT_LIMIT = 20;
    private final double ALGAE_STALL_POWER = 0.2;

    private Timer delayTimer = new Timer();
    private Timer currentTimer = new Timer();

    private WsSpark algae;
    private WsSpark coral;

    private WsDigitalInput leftShoulder;
    private WsDigitalInput rightShoulder;
    private WsAnalogInput leftTrigger;
    private WsAnalogInput rightTrigger;
    private WsDigitalInput operatorX;

    private double algaeSpeed;
    private double coralSpeed;

    private boolean algaeReefPickup;

    @Override
    public void inputUpdate(Input source) {
        if (source == operatorX) {
            algaeReefPickup = operatorX.getValue();
        } else if (source == leftShoulder) {
            coralSpeed = leftShoulder.getValue() ? 1.0 : 0.0;

            // Delay before measuring current
            if (algaeSpeed == 1.0) delayTimer.restart();
        } else if (source == rightShoulder) {
            if (algaeSpeed != 0.2) algaeSpeed = rightShoulder.getValue() ? 1 : 0;

            // Delay before measuring current
            if (algaeSpeed == 1.0) delayTimer.restart();
        } else if (source == rightTrigger) {
            if (leftTrigger.getValue() > 0.5 && rightTrigger.getValue() > 0.5) {
                if (superstructure.isReefPosition()) {
                    coralSpeed = -1;
                } else {
                    algaeSpeed = -1;
                }

            // Finish spitting out game piece
            } else if (rightTrigger.getValue() < 0.5) {
                if (algaeSpeed != 0.2) algaeSpeed = 0;
                coralSpeed = 0;
            }

        } else if (leftTrigger.getValue() > 0.5 && algaeReefPickup) {
            algaeSpeed = 1;
        }
    }

    @Override
    public void init() {
        algae = (WsSpark) WsOutputs.ALGAE_INTAKE.get();
        coral = (WsSpark) WsOutputs.CORAL_INTAKE.get();

        coral.setCurrentLimit(40,40,0);
        algae.setBrake();
        algae.setCurrentLimit(40,40,0);

        leftShoulder = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftShoulder.addInputListener(this);
        rightShoulder = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightShoulder.addInputListener(this);        
        rightTrigger = (WsAnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (WsAnalogInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        operatorX = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.OPERATOR_FACE_LEFT);
        operatorX.addInputListener(this);
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        if (coralSpeed == 1) {

            // Wait to scan current until after 0.25s to clear ramp up current spike
            if (delayTimer.hasElapsed(0.25)) {
                if (coral.getController().getOutputCurrent() < CORAL_CURRENT_LIMIT) {
                    currentTimer.reset();
                    currentTimer.stop();
                } else {
                    currentTimer.start();
                }

                // Current spike of .25s reasonable to assume picked up game piece
                if (currentTimer.hasElapsed(0.25)) {
                    coralSpeed = 0;
                }
            }
        } else if (algaeSpeed == 1.0) {

            // Wait to scan current until after 0.25s to clear ramp up current spike
            if (delayTimer.hasElapsed(0.25)) {
                if (algae.getController().getOutputCurrent() < ALGAE_CURRENT_LIMIT) {
                    currentTimer.reset();
                    currentTimer.stop();
                } else {
                    currentTimer.start();
                }

                // Current spike of .25s reasonable to assume picked up game piece
                if (currentTimer.hasElapsed(0.25)) {
                    algaeSpeed = 0.2;
                }
            }
        }
        coral.setSpeed(coralSpeed);
        algae.setSpeed(algaeSpeed);
    }

    @Override
    public void resetState() {
    }

    @Override
    public void initSubsystems() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSubsystems'");
    }

    @Override
    public String getName() {
        return "CoralPath";
    }
    
}
