package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsDigitalInput;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
//import au.grapplerobotics.LaserCan;


public class CoralPath implements Subsystem{

    private WsSpark Claw1;
    private WsSpark Coral1;


    private WsDigitalInput leftShoulder, rightShoulder, leftTrigger, rightTrigger;

    private double clawSpeed;
    private double coralSpeed;

    private boolean pickupCoral, spitCoral, pickupAlgae, spitAlgae;

    private boolean laserCoral, laserAlgae;
    
   // private LaserCan lc;



    @Override
    public void inputUpdate(Input source) {

        if (leftShoulder.getValue()) {
            pickupCoral = true;
        }
        else if (rightShoulder.getValue()) {
            pickupAlgae = true;
        }
        else if (rightTrigger.getValue() && leftTrigger.getValue()) {
            /*if (SUPERSTRUCTURE == Proccesor || SuperStrcuture == Net) { 
            runAIntake(back);
            } */
           //else {
           spitCoral = true;
            //}
        }
        else if (leftTrigger.getValue() /* && Laser Can */) {
            pickupAlgae = true;;

        }

    }

    @Override
    public void init() {
        Claw1 = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CLAW1);
        Coral1= (WsSpark) Core.getOutputManager().getOutput(WsOutputs.CORAL1);

        leftShoulder = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftShoulder.addInputListener(this);
        rightShoulder = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightShoulder.addInputListener(this);        
        rightTrigger = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (WsDigitalInput) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
    }

    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selfTest'");
    }

    @Override
    public void update() {
        if (pickupAlgae == true) {
            while (pickupAlgae) {
                clawSpeed = 1;
                if (laserAlgae == true) {
                    break;
                }
            }
        }
        else if (pickupCoral == true) {
            while (pickupCoral) {
                coralSpeed = 1;
                if (laserCoral == true) {
                    break;
                }
            }
        }
        else if (spitAlgae == true) {
            clawSpeed = -1;
        }
        else if (spitCoral == true) {
            coralSpeed = -1;
        }
    }

    @Override
    public void resetState() {
        pickupAlgae = false;
        pickupCoral = false;
        spitAlgae = false;
        spitCoral = false;
    }

    @Override
    public void initSubsystems() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initSubsystems'");
    }

    @Override
    public String getName() {
        return "CoralPath";
    }
    
}
