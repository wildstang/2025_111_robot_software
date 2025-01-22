package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsDigitalInput;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;

import com.google.gson.ToNumberPolicy;

public class Test_Climb implements Subsystem{

    private WsSpark Climb1;
    private WsDigitalInput start, select;

    private boolean shouldRun;
    private double setMode = 0;
    private int directionCombo = 0;

    private double climbSpeed;

    private double P = 0.1;
    private double I = 0;
    private double D = 0;
    private double F = 0;


    @Override
    public void inputUpdate(Input source) {

        if (start.getValue()) {
            shouldRun = true;
            setMode = 1;
        }
        else if (select.getValue()) {
            shouldRun = true;
            setMode = 2;
        }
        else if (start.getValue() && select.getValue()) {
            shouldRun = true;
            setMode = 3;
            directionCombo += 1;
        }

        }

    @Override
    public void init() {
        start = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.OPERATOR_SELECT);
        select.addInputListener(this);
        start = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.OPERATOR_START);
        select.addInputListener(this);

        Climb1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLIMB1);
        Climb1.setCurrentLimit(50, 50, 0);
        Climb1.initClosedLoop(P, I, D, F);

        resetState();
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        if (shouldRun = true) {

            if (setMode == 1) {
                climbSpeed = -1;
            }
            else if (setMode == 2) {
                climbSpeed = 1;
            }
            else if (setMode == 3) {
                if (directionCombo % 2 == 0) {
                    Climb1.setPosition(1);
                    directionCombo--;
                }
                else {
                    Climb1.setPosition(2);
                directionCombo++;
                }
            }
        }
    }

    @Override
    public void resetState() {
        setMode = 0;
        directionCombo = 0;
        shouldRun = false;
        climbSpeed = 0;
    }

    @Override
    public void initSubsystems() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSubsystems'");
    }

    @Override
    public String getName() {
        return "Test_Climb";
    }

    
}
