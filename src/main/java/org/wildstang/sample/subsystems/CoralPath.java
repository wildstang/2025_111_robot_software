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
import org.wildstang.sample.robot.CANConstants;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.robot.CANConstants;
//import au.grapplerobotics.LaserCan;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralPath implements Subsystem{


    private SuperstructureSubsystem superstructure;

    private WsSpark algae;
    private WsSpark coral;

    private WsJoystickButton leftShoulder;
    private WsJoystickButton rightShoulder;
    private WsDPadButton dpadRight;
    private WsJoystickAxis leftTrigger;
    private WsJoystickAxis rightTrigger;
    public WsLaserCAN algaeLC = new WsLaserCAN(CANConstants.ALGAE_LASERCAN);
    public WsLaserCAN coralLC = new WsLaserCAN(CANConstants.CORAL_LASERCAN);


    private double algaeSpeed;
    private double coralSpeed;
    private boolean hasCoral = false;
    private boolean intakeOverride = false;
    public WsLaserCAN lc = new WsLaserCAN(CANConstants.ALGAE_LASERCAN);


    @Override
    public void inputUpdate(Input source) {
        if (source == leftShoulder && leftShoulder.getValue()) {
            coralState = IntakeState.INTAKING;
        } else if (source == rightShoulder && rightShoulder.getValue()) {
            algaeState = IntakeState.INTAKING;
        } else if (source == rightTrigger && !superstructure.isAlgaeRemoval()) {
            
        }





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
            currentTimer.restart();
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
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        
        displayNumbers();
        lc.putData();
    }

    @Override
    public void resetState() {
    }

    @Override
    public void initSubsystems() {
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
    }

    private void displayNumbers(){
    }

    public boolean hasCoral() {

    }

    @Override
    public String getName() {
        return "CoralPath";
    }
}
