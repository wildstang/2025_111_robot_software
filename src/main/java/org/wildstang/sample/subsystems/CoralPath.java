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


    private SuperstructureSubsystem superstructure;

    private WsSpark algae;
    private WsSpark coral;

    private WsJoystickButton leftShoulder;
    private WsJoystickButton rightShoulder;
    private WsDPadButton dpadRight;
    private WsJoystickAxis leftTrigger;
    private WsJoystickAxis rightTrigger;

    private enum IntakeState { NEUTRAL, INTAKING, HOLDING, SCORING }
    private IntakeState coralState;
    private IntakeState algaeState;

    @Override
    public void inputUpdate(Input source) {
        
    }

    @Override
    public void init() {
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        
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
    }
    @Override
    public String getName() {
        return "CoralPath";
    }
}
