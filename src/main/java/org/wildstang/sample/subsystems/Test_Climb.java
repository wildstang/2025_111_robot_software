package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
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

import com.google.gson.ToNumberPolicy;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test_Climb implements Subsystem{

    private WsSpark climb1;
    private WsJoystickAxis operatorJoystickY; 
    private WsDPadButton operatorDpadDown;
    private WsDPadButton operatorDpadUp;

    private double climbSpeed;
    private double pwmValue = 0;
    private PWM servo;


    private double P = 0.1;
    private double I = 0;
    private double D = 0;
    private double F = 0;

    @Override
    public void inputUpdate(Input source) {
        if (operatorJoystickY.getValue() > 0.5) {
            climbSpeed = 1;
        } else if (operatorJoystickY.getValue() < -0.5) {
            climbSpeed = -1;
        } else {
            climbSpeed = 0;
        }
        if (source == operatorDpadDown && operatorDpadDown.getValue()) {
            pwmValue = Math.min(1, pwmValue + 0.05);
        } else if (source == operatorDpadUp && operatorDpadUp.getValue()) {
            pwmValue = Math.max(0, pwmValue - 0.05);
        }
        
    }

    @Override
    public void init() {
        servo = new PWM(6);
        operatorJoystickY = (WsJoystickAxis) WsInputs.OPERATOR_LEFT_JOYSTICK_Y.get();
        operatorJoystickY.addInputListener(this);
        operatorDpadDown = (WsDPadButton) WsInputs.OPERATOR_DPAD_DOWN.get();
        operatorDpadDown.addInputListener(this);
        operatorDpadUp = (WsDPadButton) WsInputs.OPERATOR_DPAD_UP.get();
        operatorDpadUp.addInputListener(this);

        climb1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLIMB1);
        climb1.setCurrentLimit(50, 50, 0);
        climb1.initClosedLoop(P, I, D, F);

        resetState();
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        climb1.setSpeed(climbSpeed);
        servo.setPosition(pwmValue);
        SmartDashboard.putNumber("@ pwm value", pwmValue);
        SmartDashboard.putNumber(("@ climbSpeed"), climbSpeed);
    }

    @Override
    public void resetState() {
        climbSpeed = 0;
        pwmValue = 0;
    }

    @Override
    public void initSubsystems() {
    }

    @Override
    public String getName() {
        return "Test_Climb";
    }

    
}
