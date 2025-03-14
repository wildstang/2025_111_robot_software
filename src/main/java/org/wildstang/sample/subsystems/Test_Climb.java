package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;


import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test_Climb implements Subsystem{

    private WsSpark climb1;
    private WsJoystickAxis operatorJoystickY; 
    private WsJoystickButton start, select;

    private double climbSpeed;
    private double pwmValue = 0;
    private boolean hasStarted = false;
    private PWM servo;
    private double startPos = 95;//230 for ~215 deg rotation, now 90 deg rotation
    private double position;
    private boolean manual;

    @Override
    public void inputUpdate(Input source) {
        if (operatorJoystickY.getValue() > 0.5 && hasStarted && pwmValue == 0) {
            climbSpeed = 1;
            manual = true;
        } else if (operatorJoystickY.getValue() < -0.5 && hasStarted) {
            climbSpeed = -1;
            manual = true;
        } else {
            climbSpeed = 0;
        }
        if (start.getValue() && select.getValue() && (source == start || source == select)){
            if (!hasStarted) {
                hasStarted = true;
                manual = false;
                position = startPos;
            }
            else if (pwmValue == 0) pwmValue = 0.55;
            else pwmValue = 0;
        }
        
    }

    @Override
    public void init() {
        servo = new PWM(6);
        operatorJoystickY = (WsJoystickAxis) WsInputs.OPERATOR_LEFT_JOYSTICK_Y.get();
        operatorJoystickY.addInputListener(this);
        start = (WsJoystickButton) WsInputs.OPERATOR_START.get();
        start.addInputListener(this);
        select = (WsJoystickButton) WsInputs.OPERATOR_SELECT.get();
        select.addInputListener(this);

        climb1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLIMB1);
        climb1.setCurrentLimit(50, 50, 0);

        resetState();
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        if (!manual) {
            if (climb1.getPosition() < position) climbSpeed = 1;
            else climbSpeed = 0;
            if (climb1.getPosition() > startPos) pwmValue = 0.55;
        }
        climb1.setSpeed(climbSpeed);
        servo.setPosition(pwmValue);
        SmartDashboard.putBoolean("# climb ratchet on", pwmValue == 0.55);
        SmartDashboard.putNumber("@ pwm value", pwmValue);
        SmartDashboard.putNumber("@ climbSpeed", climbSpeed);
        SmartDashboard.putBoolean("@ climb started", hasStarted);
        SmartDashboard.putNumber("@ climb position", climb1.getPosition());
    }

    @Override
    public void resetState() {
        climbSpeed = 0;
        pwmValue = 0;
        manual = false;
        position = 15;
    }

    @Override
    public void initSubsystems() {
    }

    @Override
    public String getName() {
        return "Test_Climb";
    }

    
}
