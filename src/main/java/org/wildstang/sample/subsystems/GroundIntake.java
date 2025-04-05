package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsDPadButton;
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
    private CoralPath coralPath;

    private final double STARTING = -23.1;
    private final double L1 = -20;
    private final double L1Score = -15;
    private final double DEPLOYED = 0;
    private final double CLIMB = -8;
    private double ground1Speed = 0;
    private double ground2Speed = 0;
    private Timer L1timer = new Timer();
    private double deploy = STARTING;
    private enum IntakeState {NEUTRAL, INTAKING, INTAKING_L1, REVERSE, PRE_L1, SCORE_L1, CLIMB, UP, STATION, AUTO};
    private IntakeState state = IntakeState.AUTO;

    @Override
    public void inputUpdate(Input source){

        // After 1 second of clicking start and select, bring up intake slightly
        if (start.getValue() && select.getValue() && (source == start || source == select)){
            state = IntakeState.CLIMB;
        }
        if (state == IntakeState.CLIMB) ;//nothing
        else if (Math.abs(rightTrigger.getValue()) > 0.5){
            if (superstructure.isScoreL1() && Math.abs(leftTrigger.getValue()) > 0.5){
                state = IntakeState.SCORE_L1;
            }
            else if (Math.abs(leftTrigger.getValue()) < 0.5){
                state = IntakeState.INTAKING;
            }
        } else if (Math.abs(leftTrigger.getValue()) > 0.5 && superstructure.isScoreL1()){
            if (state != IntakeState.PRE_L1) L1timer.reset();
            state = IntakeState.PRE_L1;
        } else if (DpadUp.getValue() || driverDPadLeft.getValue()){
            state = IntakeState.REVERSE;
        } else if (leftShoulder.getValue()){
            state = IntakeState.STATION;
        } else if (rightShoulder.getValue()){
            state = IntakeState.INTAKING_L1;
        } else if (dpadDown.getValue()){
            state = IntakeState.UP;
        } else {
            state = IntakeState.NEUTRAL;
        }
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
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        if (state == IntakeState.CLIMB){
            deploy = superstructure.isAtPosition() ? CLIMB : DEPLOYED;
            ground1Speed = 0;
            ground2Speed = 0;
        } else if (state == IntakeState.INTAKING){
            deploy = DEPLOYED;
            if (coralPath.hasCoral()){
                ground1Speed = 0;
                ground2Speed = 0;
            } else {
                ground1Speed = superstructure.isAtPosition() ? 1 : 0;
                ground2Speed = -1;
            }
        } else if (state == IntakeState.INTAKING_L1){
            deploy = DEPLOYED;
            ground1Speed = -1;
            ground2Speed = -1;
        } else if (state == IntakeState.NEUTRAL){
            deploy = superstructure.isScoreL1() ? L1 : DEPLOYED;
            ground1Speed = 0;
            ground2Speed = superstructure.isScoreL1() ? -0.2 : 0;
        } else if (state == IntakeState.REVERSE){
            deploy = DEPLOYED;
            ground1Speed = -1;
            ground2Speed = 0.25;
        } else if (state == IntakeState.SCORE_L1){
            deploy = L1Score;
            ground1Speed = -1;
            ground2Speed = 0.25;
        } else if (state == IntakeState.PRE_L1){
            deploy = L1;
            ground1Speed = L1timer.hasElapsed(0.2) ? 0.15 : 0;
            ground2Speed = 0;
        } else if (state == IntakeState.STATION){
            deploy = superstructure.isAtPosition() ? STARTING : DEPLOYED;
            ground1Speed = 0;
            ground2Speed = 0;
        } else if (state == IntakeState.UP){
            deploy = STARTING;
            ground1Speed = 0;
            ground2Speed = 0;
        }
        pivot.setPosition(deploy);
        ground1.setSpeed(ground1Speed);
        ground2.setSpeed(ground2Speed);
        SmartDashboard.putNumber("@ Intake position", pivot.getPosition());
        SmartDashboard.putNumber("@ Intake target", deploy);
        SmartDashboard.putString("@ Ground Intake State", state.toString());
    }

    @Override
    public void resetState() {
    }

    public void deploy() {
        deploy = DEPLOYED;
        state = IntakeState.AUTO;
    }
    public void stow() {
        deploy = STARTING;
        state = IntakeState.AUTO;
    }

    public void groundOn() {
        ground1Speed = 1;
        ground2Speed = -1;
    }
    public void groundOff(){
        ground1Speed = 0;
        ground2Speed = 0;
    }
    public void groundL1(){
        ground1Speed = -1;
        ground2Speed = 0.25;
    }

    @Override
    public String getName() {
        return "Ground Intake";
    }
}
