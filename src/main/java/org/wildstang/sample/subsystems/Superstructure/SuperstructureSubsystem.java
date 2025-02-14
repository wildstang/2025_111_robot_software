package org.wildstang.sample.subsystems.Superstructure;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.hardware.roborio.inputs.WsDPadButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.swerve.WsSwerveHelper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.swerve.SwerveDrive; 


public class SuperstructureSubsystem implements Subsystem {

private WsJoystickButton  LShoulder,Rshoulder,A,B,Y,X,Start,Select;
private WsJoystickAxis LT,operator_RT, operator_LT;
private WsDPadButton DPad_UP, DPad_LEFT, DPad_DOWN;
private boolean LShoulderHeld,RshoulderHeld,StartHeld,SelectHeld,LTHeld;
private boolean topTriangle, PickupSequence;
public SuperstructurePosition desiredPosition = SuperstructurePosition.STOWED;
private SuperstructurePosition prevPosition = SuperstructurePosition.STOWED;
private WsSpark LiftMax, lift2, armSpark ;
private double initialAbsolute = 0;
private SwerveDrive swerve;
private CoralPath coralPath;
private final double LIFT_FF = 0;
private final double BACK_CLEAR = 72;
private final double FRONT_CLEAR = 44;
private final double LIFT_STAGE1 = 50;
private final double LIFT_LOW = 25;


private enum LevelReef{
Reef_L1,
Reef_L2,
Reef_L3,
Reef_L4,
}
private enum Algae_NetOrProces{
    Net,
    Processor,
}
LevelReef level = LevelReef.Reef_L1;
private String[] reefLevelNames = {"L1", "L2", "L3", "L4"};
Algae_NetOrProces AlgaeState = Algae_NetOrProces.Net;


@Override
    public void init() {
        LShoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        LShoulder.addInputListener(this);
        Rshoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        Rshoulder.addInputListener(this);
        LT = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        LT.addInputListener(this);
        A = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_FACE_DOWN);
        A.addInputListener(this);
        B = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_FACE_RIGHT);
        B.addInputListener(this);
        Y = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_FACE_UP);
        Y.addInputListener(this);
        X = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_FACE_LEFT);
        X.addInputListener(this);
        DPad_UP = (WsDPadButton) Core.getInputManager().getInput(WsInputs.OPERATOR_DPAD_UP);
        DPad_UP.addInputListener(this);
        DPad_LEFT = (WsDPadButton) Core.getInputManager().getInput(WsInputs.OPERATOR_DPAD_LEFT);
        DPad_LEFT.addInputListener(this);
        DPad_DOWN = (WsDPadButton) Core.getInputManager().getInput(WsInputs.OPERATOR_DPAD_DOWN);
        DPad_DOWN.addInputListener(this);
        Start = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_START);
        Start.addInputListener(this);
        Select = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_SELECT);
        Select.addInputListener(this);
        operator_RT = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.OPERATOR_RIGHT_TRIGGER);
        operator_RT.addInputListener(this);
        operator_LT = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.OPERATOR_LEFT_TRIGGER);
        operator_LT.addInputListener(this);
       
        armSpark = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARM);
        armSpark.initClosedLoop(0.15, 0, 0, 0);
        armSpark.addClosedLoop(1, 0.05, 0, 0, 0);
        initialAbsolute = armSpark.getController().getAbsoluteEncoder().getPosition();
        //armSpark.setPosition((initialAbsolute - ABS_ZERO) / (11.25/360));//9 72, 18 72, 16 68 means 32 motor rot per arm rot, 11.25 deg per rot
        armSpark.setBrake();
        armSpark.setCurrentLimit(20, 20, 0);
        LiftMax = (WsSpark) WsOutputs.LIFT.get();
        lift2 = (WsSpark) WsOutputs.LIFT_FOLLOWER.get();
        LiftMax.initClosedLoop(1.0,0,0,0);
        lift2.initClosedLoop(1.0,0,0,0);
        LiftMax.addClosedLoop(1, 0.1, 0.0, 0.05, 0);
        lift2.addClosedLoop(1, 0.1, 0, 0.05, 0);
        LiftMax.setBrake();
        lift2.setBrake();
        lift2.setPosition(LiftMax.getPosition());
        LiftMax.setCurrentLimit(65, 65, 0);
        lift2.setCurrentLimit(65, 65, 0);

    }

@Override
    public void update() {

        
        if (LShoulderHeld) {
            if (swerve.isCoralStationFront()) {
                desiredPosition = SuperstructurePosition.CORAL_STATION_FRONT;
            } else {
                desiredPosition = SuperstructurePosition.CORAL_STATION_BACK;
            }
        } 
        else if (RshoulderHeld) {
            desiredPosition = SuperstructurePosition.ALGAE_PRESTAGED;
        } 
        else if (LTHeld) {
            if ((coralPath.hasAlgae() || isScoringAlgae()) && !(coralPath.hasCoral() || isScoringCoral()) && !PickupSequence) {
                if (Algae_NetOrProces.Net == AlgaeState) {
                    if (swerve.isNetFront()){
                        desiredPosition = SuperstructurePosition.ALGAE_NET_FRONT;
                    } else {
                        desiredPosition = SuperstructurePosition.ALGAE_NET_BACK;
                    }
                } else if (Algae_NetOrProces.Processor == AlgaeState) {
                    if(swerve.isProcessorFront()){
                        desiredPosition = SuperstructurePosition.ALGAE_PROCESSOR_FRONT;
                    } else{ 
                        desiredPosition = SuperstructurePosition.ALGAE_PROCESSOR_BACK;
                    }
                }
            } else if (PickupSequence) {
                if (topTriangle) desiredPosition = SuperstructurePosition.ALGAE_REEF_LOW;
                else desiredPosition = SuperstructurePosition.ALGAE_REEF_HIGH;
                if (coralPath.hasAlgae()) {
                    if (topTriangle) desiredPosition = SuperstructurePosition.STOWED_AFTER_PICKUP_LOW;
                    else desiredPosition = SuperstructurePosition.STOWED_AFTER_PICKUP_HIGH;
                    if (armAtPosition()) PickupSequence = false;
                }
            } 
            else {
                if (LevelReef.Reef_L1 == level) {
                    desiredPosition = SuperstructurePosition.CORAL_REEF_L1;
                } else if (LevelReef.Reef_L2 == level) {
                    desiredPosition = SuperstructurePosition.CORAL_REEF_L2;
                } else if (LevelReef.Reef_L3 == level) {
                    desiredPosition = SuperstructurePosition.CORAL_REEF_L3;
                } else if (LevelReef.Reef_L4 == level) {
                    desiredPosition = SuperstructurePosition.CORAL_REEF_L4;
                }
            }
        } 
        else if (StartHeld && SelectHeld) {
            desiredPosition = SuperstructurePosition.CLIMB;
        } else {
            desiredPosition = SuperstructurePosition.STOWED;
        }
        if (prevPosition != desiredPosition && armAtPosition() && liftAtPosition()){
            prevPosition = desiredPosition;
        }

        if (!liftAtPosition() && !isArmClear(desiredPosition) && isScoringCoral()){
            setArm(getClearArm(desiredPosition));
        } else {
            setArm(desiredPosition.getArm());
        }

        if (!isArmClear(armSpark.getPosition()) && !armAtPosition() && isReefPosition(prevPosition)){
            setLift(LiftMax.getPosition());
        } else {
            setLift(desiredPosition.getLift());
        }

        displayNumbers();
}

    @Override
    public void inputUpdate(Input source) {
        LShoulderHeld = LShoulder.getValue();
        RshoulderHeld = Rshoulder.getValue();
        LTHeld = Math.abs(LT.getValue()) > 0.5;
        if (Math.abs(operator_LT.getValue()) > 0.5) topTriangle = false;
        if (Math.abs(operator_RT.getValue()) > 0.5) topTriangle = true;
        if(A.getValue()){
            level = LevelReef.Reef_L2;
        }
        if(B.getValue()){
            level = LevelReef.Reef_L3;
        }
        if (Y.getValue()) {
            level = LevelReef.Reef_L4;
        }
        if (source == X && X.getValue()) PickupSequence = !PickupSequence;
        if(DPad_UP.getValue()){
           AlgaeState = Algae_NetOrProces.Net;
        }
        if(DPad_LEFT.getValue()){
            AlgaeState = Algae_NetOrProces.Processor;
        }
        if(DPad_DOWN.getValue()){
            level = LevelReef.Reef_L1;
        }
        StartHeld = Start.getValue();
        SelectHeld = Select.getValue();
        if (desiredPosition == SuperstructurePosition.CLIMB){
            StartHeld = true;
            SelectHeld = true;
        }

        
    }
    @Override
    public void initSubsystems() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
    }
    @Override
    public void selfTest() {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void resetState() {
        PickupSequence = false;
        desiredPosition = SuperstructurePosition.STOWED;
        level = LevelReef.Reef_L3;
        AlgaeState = Algae_NetOrProces.Processor;
        
    }
    @Override
    public String getName() {
        return "Superstructure Subsystem";
    }
    private void displayNumbers(){
        SmartDashboard.putBoolean("# Algae Pickup", PickupSequence);
        SmartDashboard.putBoolean("# Top Triangle", topTriangle);
        SmartDashboard.putBoolean("# Bottom Triangle", !topTriangle);
        SmartDashboard.putNumber("@ Arm Target", desiredPosition.getArm());
        SmartDashboard.putNumber("@ Arm Position", armSpark.getPosition());
        SmartDashboard.putNumber("@ Lift Target", desiredPosition.getLift());
        SmartDashboard.putNumber("@ Lift Position", LiftMax.getPosition());
        SmartDashboard.putString("@ Superstructure position", desiredPosition.getName());
        SmartDashboard.putBoolean("# Targeting Net", AlgaeState == Algae_NetOrProces.Net);
        SmartDashboard.putBoolean("# Targeting Processor", AlgaeState == Algae_NetOrProces.Processor);
        SmartDashboard.putString("# Reef Level", reefLevelNames[level.ordinal()]);
        SmartDashboard.putBoolean("# Reef L1", level.ordinal() == 0);
        SmartDashboard.putBoolean("# Reef L2", level.ordinal() == 1);
        SmartDashboard.putBoolean("# Reef L3", level.ordinal() == 2);
        SmartDashboard.putBoolean("# Reef L4", level.ordinal() == 3);
        SmartDashboard.putNumber("@ absolute arm", armSpark.getController().getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("@ lift current", LiftMax.getController().getOutputCurrent());
    }

     /**
     * @return true if pickupSequence is on ||
    *  false each if pickupSequence is off
    * 
    */
    public boolean isAlgaeRemoval(){
        return PickupSequence;
    }
    private boolean liftAtPosition(){
        return Math.abs(desiredPosition.getLift() - LiftMax.getPosition())<4;
    }
    private boolean armAtPosition(){
        return Math.abs(desiredPosition.getArm() - armSpark.getPosition())<2;
    }
    public boolean isAtPosition(){
        return liftAtPosition() && armAtPosition();
    }
    /**
     * @return true if Net ||
    *  false Proccesor 
    * 
    */
    public boolean isAlgaeNet(){
        return (Algae_NetOrProces.Net == AlgaeState);    
    }
    public boolean isScoringAlgae(){
        return desiredPosition == SuperstructurePosition.ALGAE_NET_BACK || 
            desiredPosition == SuperstructurePosition.ALGAE_NET_FRONT || 
            desiredPosition == SuperstructurePosition.ALGAE_PROCESSOR_BACK || 
            desiredPosition == SuperstructurePosition.ALGAE_PROCESSOR_FRONT;
    }
    public boolean isScoringCoral(){
        return desiredPosition == SuperstructurePosition.CORAL_REEF_L1 || 
            desiredPosition == SuperstructurePosition.CORAL_REEF_L2 ||
            desiredPosition == SuperstructurePosition.CORAL_REEF_L3 || 
            desiredPosition == SuperstructurePosition.CORAL_REEF_L4 ||
            desiredPosition == SuperstructurePosition.STOWED_AFTER_PICKUP_HIGH ||
            desiredPosition == SuperstructurePosition.STOWED_AFTER_PICKUP_LOW;
    }
    private void setArm(double armPos){
        if (isLiftHigh(LiftMax.getPosition()) || isLiftHigh(desiredPosition)){
            armSpark.setPosition(armPos, 1);
        } else {
            armSpark.setPosition(armPos, 0);
        }
    }
    private void setLift(double liftPos){
        // if (desiredPosition.getLift() < LIFT_LOW && LiftMax.getPosition() < LIFT_LOW){
        if (desiredPosition.getLift() < LiftMax.getPosition()){
            LiftMax.setPosition(liftPos, 1, LIFT_FF);
            lift2.setPosition(-liftPos, 1, -LIFT_FF);
        } else {
            LiftMax.setPosition(liftPos, 0, LIFT_FF);
            lift2.setPosition(-liftPos, 0, -LIFT_FF);
        }
    }
    private boolean isLiftHigh(SuperstructurePosition position){
        return isLiftHigh(position.getLift());
    }
    private boolean isLiftHigh(double position){
        return position > LIFT_STAGE1;
    }
    private boolean isArmClear(SuperstructurePosition position){
        return isArmClear(position.getArm());
    }
    private boolean isArmClear(double position){
        return position < BACK_CLEAR && position > FRONT_CLEAR;
    }
    private double getClearArm(SuperstructurePosition position){
        if (position.getArm() > BACK_CLEAR) return BACK_CLEAR;
        else return FRONT_CLEAR;
    }
    private boolean isReefPosition(SuperstructurePosition position){
        return position == SuperstructurePosition.CORAL_REEF_L1 || 
            position == SuperstructurePosition.CORAL_REEF_L2 ||
            position == SuperstructurePosition.CORAL_REEF_L3 || 
            position == SuperstructurePosition.CORAL_REEF_L4;
    }

}
