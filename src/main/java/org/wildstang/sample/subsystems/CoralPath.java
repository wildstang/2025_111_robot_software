package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsAnalogInput;
import org.wildstang.hardware.roborio.inputs.WsDPadButton;
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
    private static final double ALGAE_CURRENT_LIMIT = 40;
    private final double ALGAE_STALL_POWER = 0.95;

    private Timer delayTimer = new Timer();
    private Timer currentTimer = new Timer();
    private Timer holdTimer = new Timer();

    private SuperstructureSubsystem superstructure;

    private WsSpark algae;
    private WsSpark coral;

    private WsJoystickButton leftShoulder;
    private WsJoystickButton rightShoulder;
    private WsDPadButton dpadRight;
    private WsJoystickAxis leftTrigger;
    private WsJoystickAxis rightTrigger;

    private double algaeSpeed;
    private double coralSpeed;
    private boolean hasCoral = false;
    private boolean intakeOverride = false;


    @Override
    public void inputUpdate(Input source) {
        intakeOverride = leftShoulder.getValue();
        if (source == leftShoulder) {
            coralSpeed = leftShoulder.getValue() ? 1.0 : 0.0;

            // Delay before measuring current
            if (coralSpeed == 1.0) delayTimer.restart();
            if (!leftShoulder.getValue()) hasCoral = true;
        } else if (source == rightShoulder) {
            if (algaeSpeed != ALGAE_STALL_POWER) algaeSpeed = rightShoulder.getValue() ? 1 : 0;

            // Delay before measuring current
            if (algaeSpeed == 1.0) delayTimer.restart();
        } else if (source == rightTrigger && !superstructure.isAlgaeRemoval()) {
            if (Math.abs(leftTrigger.getValue()) > 0.5 && Math.abs(rightTrigger.getValue()) > 0.5) {
                if (!hasAlgae() || hasCoral()) {
                    if (superstructure.isScoreL1()) coralSpeed = -0.4;
                    else if (superstructure.isScoreL23()) coralSpeed = -0.7;//-0.6 for med wheels
                    else coralSpeed = -1.0;
                } else {
                    algaeSpeed = -1;
                }

            // Finish spitting out game piece
            } else if (rightTrigger.getValue() < 0.5 && !superstructure.isAlgaeRemoval()) {
                if (algaeSpeed == -1) algaeSpeed = 0;
                if (coralSpeed == -0.4 || coralSpeed == -1.0 || coralSpeed == -0.7){//-0.6 for med wheels
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
        if (source == dpadRight && dpadRight.getValue()){
            algaeSpeed = 0;
        }
    }

    @Override
    public void init() {
        algae = (WsSpark) WsOutputs.ALGAE_INTAKE.get();
        coral = (WsSpark) WsOutputs.CORAL_INTAKE.get();

        coral.setBrake();
        coral.setCurrentLimit(60,60,0);//60 for med wheels
        algae.setBrake();
        algae.setCurrentLimit(60,60,0);

        leftShoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftShoulder.addInputListener(this);
        rightShoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightShoulder.addInputListener(this);        
        rightTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        dpadRight = (WsDPadButton) WsInputs.OPERATOR_DPAD_RIGHT.get();
        dpadRight.addInputListener(this);
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
            if (delayTimer.hasElapsed(0.75)) {
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
            if (delayTimer.hasElapsed(1.0)) {
                if (algae.getController().getOutputCurrent() < ALGAE_CURRENT_LIMIT) {
                    currentTimer.reset();
                    currentTimer.stop();
                } else {
                    currentTimer.start();
                }

                // Current spike of .25s reasonable to assume picked up game piece
                if (currentTimer.hasElapsed(0.25)) {
                    algaeSpeed = ALGAE_STALL_POWER;
                    holdTimer.restart();
                }
            }
        }
        // if (algae.getController().getOutputCurrent() > ALGAE_CURRENT_LIMIT && algaeSpeed == 0 && delayTimer.hasElapsed(0.25)){
        //     algaeSpeed = ALGAE_STALL_POWER;
        // }
        if (intakeOverride) coral.setSpeed(1.0);
        else coral.setSpeed(coralSpeed);
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
        return algaeSpeed == ALGAE_STALL_POWER || algaeSpeed == -1.0 || superstructure.isScoringAlgae()
            || (algaeSpeed == 1.0 && !holdTimer.hasElapsed(2));
    }
    public boolean hasCoral(){
        return hasCoral || superstructure.isScoringCoral();
    }

    // AUTO STEP METHODS

    // Start or stop intaking coral
    public void setIntake(boolean intake) {
        coralSpeed = intake ? 1 : 0;
        delayTimer.start();
    }

    // Start or stop scoring coral
    public void setScore(boolean score) {
        coralSpeed = score ? -1 : 0;
    }
    public void scored(){
        hasCoral = false;
    }
    
}
