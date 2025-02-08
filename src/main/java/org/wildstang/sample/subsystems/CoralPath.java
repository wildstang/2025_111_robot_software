package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsAnalogInput;
import org.wildstang.hardware.roborio.inputs.WsDigitalInput;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
//import au.grapplerobotics.LaserCan;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralPath implements Subsystem{

    private static final double CORAL_CURRENT_LIMIT = 20;
    private static final double ALGAE_CURRENT_LIMIT = 25;
    private final double ALGAE_STALL_POWER = 0.5;

    private Timer delayTimer = new Timer();
    private Timer currentTimer = new Timer();
    private Timer holdTimer = new Timer();

    private SuperstructureSubsystem superstructure;

    private WsSpark algae;
    private WsSpark coral;

    private WsJoystickButton leftShoulder;
    private WsJoystickButton rightShoulder;
    private WsJoystickAxis leftTrigger;
    private WsJoystickAxis rightTrigger;

    private double algaeSpeed;
    private double coralSpeed;
    private boolean hasCoral = false;


    @Override
    public void inputUpdate(Input source) {
        if (source == leftShoulder) {
            coralSpeed = leftShoulder.getValue() ? 1.0 : 0.0;

            // Delay before measuring current
            if (coralSpeed == 1.0) delayTimer.restart();
        } else if (source == rightShoulder) {
            if (algaeSpeed != ALGAE_STALL_POWER) algaeSpeed = rightShoulder.getValue() ? 1 : 0;

            // Delay before measuring current
            if (algaeSpeed == 1.0) delayTimer.restart();
        } else if (source == rightTrigger && !superstructure.isAlgaeRemoval()) {
            if (Math.abs(leftTrigger.getValue()) > 0.5 && Math.abs(rightTrigger.getValue()) > 0.5) {
                if (!hasAlgae() || hasCoral()) {
                    coralSpeed = -1;
                } else {
                    algaeSpeed = -1;
                }

            // Finish spitting out game piece
            } else if (rightTrigger.getValue() < 0.5 && !superstructure.isAlgaeRemoval()) {
                if (algaeSpeed == -1) algaeSpeed = 0;
                if (coralSpeed == -1){
                    coralSpeed = 0;
                    hasCoral = false;
                }
            }

        } else if (leftTrigger.getValue() > 0.5 && superstructure.isAlgaeRemoval()) {
            algaeSpeed = 1;
            delayTimer.restart();
        } else if (Math.abs(leftTrigger.getValue()) < 0.5 && !hasAlgae()){
            algaeSpeed = 0;
        }
    }

    @Override
    public void init() {
        algae = (WsSpark) WsOutputs.ALGAE_INTAKE.get();
        coral = (WsSpark) WsOutputs.CORAL_INTAKE.get();

        coral.setBrake();
        coral.setCurrentLimit(40,40,0);
        algae.setBrake();
        algae.setCurrentLimit(50,50,0);

        leftShoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftShoulder.addInputListener(this);
        rightShoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightShoulder.addInputListener(this);        
        rightTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        // currentTimer.start();
        // delayTimer.start();
        holdTimer.start();
    }

    @Override
    public void selfTest() {
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
                    hasCoral = true;
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
                if (currentTimer.hasElapsed(0.5)) {
                    algaeSpeed = ALGAE_STALL_POWER;
                    holdTimer.restart();
                }
            }
        }
        coral.setSpeed(coralSpeed);
        if (algaeSpeed == ALGAE_STALL_POWER && !holdTimer.hasElapsed(2.0)){
            algae.setSpeed(1);
        } else {
            algae.setSpeed(algaeSpeed);
        }

        displayNumbers();
    }

    @Override
    public void resetState() {
    }

    @Override
    public void initSubsystems() {
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
    }

    private void displayNumbers(){
        SmartDashboard.putBoolean("# Has Coral", hasCoral());
        SmartDashboard.putBoolean("# Has Algae", hasAlgae());
        SmartDashboard.putNumber("@ Coral Speed", coralSpeed);
        SmartDashboard.putNumber("@ Alage Speed", algaeSpeed);
        SmartDashboard.putNumber("@ Coral Current", coral.getController().getOutputCurrent());
        SmartDashboard.putNumber("@ Algae Current", algae.getController().getOutputCurrent());
        SmartDashboard.putNumber("@ delay timer", delayTimer.get());
        SmartDashboard.putNumber("@ current timer", currentTimer.get());
    }
    @Override
    public String getName() {
        return "CoralPath";
    }
    public boolean hasAlgae(){
        return algaeSpeed == ALGAE_STALL_POWER || algaeSpeed == -1.0 || superstructure.isScoringAlgae();
    }
    public boolean hasCoral(){
        return hasCoral || superstructure.isScoringCoral();
    }
    
}
