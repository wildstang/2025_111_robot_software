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
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GroundIntake implements Subsystem {
   
    private WsSpark pivot;
    private WsSpark ground1;
    private WsSpark ground2;

    private WsJoystickAxis leftTrigger;
    private WsJoystickAxis rightTrigger;
    private WsJoystickButton leftShoulder;
    private WsJoystickButton rightShoulder;
    private WsJoystickButton start, select;
    private WsDPadButton DpadUp;
    private WsDPadButton dpadDown;
    private WsDPadButton driverDPadLeft;

    private SuperstructureSubsystem superstructure;

    private final double STARTING = -23.1;
    private final double L1 = -19;
    private final double L1Score = -16;
    private final double DEPLOYED = 0;
    private final double CLIMB = -12;
    private double ground1Speed;
    private double ground2Speed;
    private double deploy = STARTING; 
    private Timer L1timer = new Timer();

    // After 1 second of clicking start and select, bring up intake slightly
    private Timer climbTimer = new Timer();


    @Override
    public void inputUpdate(Input source){

        // After 1 second of clicking start and select, bring up intake slightly
        if (start.getValue() && select.getValue() && (source == start || source == select)){
            climbTimer.start();
        }
        if (source == dpadDown && dpadDown.getValue()) {
            if (deploy == DEPLOYED) deploy = STARTING;
            if (deploy == STARTING) deploy = DEPLOYED;
        }
        if (Math.abs(rightTrigger.getValue()) > 0.5){
            if (superstructure.isScoreL1() && Math.abs(leftTrigger.getValue()) > 0.5){
                //score L1
                ground1Speed = -1;
                ground2Speed = 0.25;
            }
            else if (Math.abs(leftTrigger.getValue()) < 0.5){
                //nomral intake
                ground1Speed = 1;
                ground2Speed = -1;
            }
        } else if (DpadUp.getValue() || driverDPadLeft.getValue()){
            //reverse
            ground1Speed = -1;
            ground2Speed = 0.25;
        } else if (rightShoulder.getValue()){
            //intake for L1
            ground1Speed = -1;
            ground2Speed = -1;
        } else if (superstructure.isScoreL1()){
            //hold in L1
            ground1Speed = 0;
            ground2Speed = -0.2;
        } else {
            ground1Speed = 0;
            ground2Speed = 0;
        }
       if (Math.abs(leftTrigger.getValue()) > 0.5 && superstructure.isScoreL1()){
            if (Math.abs(rightTrigger.getValue()) > 0.5) {
                deploy = L1Score;
            } else {
                if (deploy != L1) L1timer.reset(); 
                deploy = L1;
            }
       } else if (leftShoulder.getValue()){
        deploy = STARTING;
       } else deploy = DEPLOYED;
    }  

    @Override
    public void init() {
        ground1 = (WsSpark) WsOutputs.GROUND1.get();
        ground2 = (WsSpark) WsOutputs.GROUND2.get();
        pivot = (WsSpark) WsOutputs.PIVOT.get();
        pivot.initClosedLoop(0.1,0,0,0);
        ground1.setBrake();
        ground2.setBrake();
        pivot.setBrake();
        ground1.setCurrentLimit(50, 50, 0);
        ground2.setCurrentLimit(50, 50, 0);
        pivot.setCurrentLimit(50, 50, 0);

        rightTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        leftShoulder = (WsJoystickButton) WsInputs.DRIVER_LEFT_SHOULDER.get();
        leftShoulder.addInputListener(this);
        rightShoulder = (WsJoystickButton) WsInputs.DRIVER_RIGHT_SHOULDER.get();
        rightShoulder.addInputListener(this);
        DpadUp = (WsDPadButton) WsInputs.OPERATOR_DPAD_UP.get();
        DpadUp.addInputListener(this);
        dpadDown = (WsDPadButton) WsInputs.OPERATOR_DPAD_DOWN.get();
        dpadDown.addInputListener(this);
        driverDPadLeft = (WsDPadButton) WsInputs.DRIVER_DPAD_LEFT.get();
        driverDPadLeft.addInputListener(this);
        start = (WsJoystickButton) WsInputs.OPERATOR_START.get();
        start.addInputListener(this);
        select = (WsJoystickButton) WsInputs.OPERATOR_SELECT.get();
        select.addInputListener(this);
        
        L1timer.start();
    }

    @Override
    public void initSubsystems() {
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        if (climbTimer.hasElapsed(1)) {
            pivot.setPosition(CLIMB);
        } else {
            if (!L1timer.hasElapsed(0.1)){
                ground1.setSpeed(0.15);
            } else {
                ground1.setSpeed(ground1Speed);
            }
            ground2.setSpeed(ground2Speed);
    
            pivot.setPosition(deploy);
        }
        SmartDashboard.putNumber("@ Intake position", pivot.getPosition());
        SmartDashboard.putNumber("@ Intake target", deploy);
    }

    @Override
    public void resetState() {
        ground1Speed = 0;
        ground2Speed = 0;
    }

    public void deploy() {
        deploy = DEPLOYED;
    }
    public void stow() {
        deploy = STARTING;
    }

    public void groundOn() {
        ground1Speed = 1;
        ground2Speed = -1;
    }

    @Override
    public String getName() {
        return "Ground Intake";
    }
}
