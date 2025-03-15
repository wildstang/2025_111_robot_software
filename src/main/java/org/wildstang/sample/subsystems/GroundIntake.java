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

public class GroundIntake implements Subsystem {
   
    private WsSpark pivot;
    private WsSpark ground1;
    private WsSpark ground2;

    private WsJoystickAxis leftTrigger;
    private WsJoystickAxis rightTrigger;

    private SuperstructureSubsystem superstructure;

    private double deployed;
    private double starting;
    private double ground1Speed;
    private double ground2Speed;
    private boolean deploy;


    @Override
    public void inputUpdate(Input source){
        if (source == rightTrigger && Math.abs(rightTrigger.getValue()) > 0.5){
            if(Math.abs(leftTrigger.getValue()) < 0.5 && superstructure.isScoreL1()){
                ground1Speed = 1;
                ground2Speed = -1;
            }
            else if (superstructure.isScoreL1()){
                ground1Speed = -1;
                ground2Speed = -1;
            }
            else if (Math.abs(leftTrigger.getValue()) < 0.5){
                ground1Speed = 1;
                ground2Speed = 1;
            }
        }
       if (source == leftTrigger && Math.abs(leftTrigger.getValue()) > 0.5){
          if (superstructure.isScoreL1()){
            deploy = false;
          }
       }
       else deploy = true;
    }  

    @Override
    public void init() {
        ground1 = (WsSpark) WsOutputs.GROUND1.get();
        ground2 = (WsSpark) WsOutputs.GROUND2.get();
        pivot = (WsSpark) WsOutputs.PIVOT.get();
        pivot.initClosedLoop(1.0,0,0,0);

        rightTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        
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
        ground1.setSpeed(ground1Speed);
        ground2.setSpeed(ground2Speed);

        if (deploy){
            pivot.setPosition(deployed);
        }
        else pivot.setPosition(starting);
    }

    @Override
    public void resetState() {
        ground1Speed = 0;
        ground2Speed = 0;
        deploy = true;
    }

    public void deploy() {
        deploy = true;
    }

    public void groundOn() {
        ground1Speed = 1;
        ground2Speed = 1;
    }

    @Override
    public String getName() {
        return "Ground Intake";
    }
}
