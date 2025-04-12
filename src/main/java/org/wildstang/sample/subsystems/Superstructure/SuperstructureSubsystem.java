package org.wildstang.sample.subsystems.Superstructure;

import org.wildstang.framework.auto.AutoStep;
import org.wildstang.framework.auto.steps.LambdaStep;
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
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;
import org.wildstang.sample.subsystems.targeting.WsPose;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 


public class SuperstructureSubsystem implements Subsystem {

    public static AutoStep setPositionStep(SuperstructurePosition position) {
        return new LambdaStep(() -> {
           SuperstructureSubsystem superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
           superstructure.setPosition(position);
        }, "Set Superstructure Position Step");
    };

private WsJoystickButton  LShoulder,Rshoulder,A,B,Y,X,Start,Select, reset;
private WsJoystickAxis LT,operator_RT, operator_LT, rightTrigger, resetAxis;
private WsDPadButton operatorDleft,operatorDright;
private boolean LShoulderHeld,StartHeld,SelectHeld,LTHeld, RTHeld, dRightHeld;
private boolean PickupSequence;
private boolean resetStatus = false;
private double resetSpeed = 0.0;
public SuperstructurePosition desiredPosition = SuperstructurePosition.STOWED;
private SuperstructurePosition prevPosition = SuperstructurePosition.CORAL_STATION_FRONT;
private WsSpark LiftMax, lift2, armSpark ;
private boolean isAuto = true;
private boolean isLevel1 = false;
private boolean override = false;
private SwerveDrive swerve;
private CoralPath coralPath;
private WsPose pose;
private final double LIFT_FF = 0;
private final double BACK_CLEAR = 68;
private final double LIFT_STAGE1 = 50 *5.0/9.0;


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
LevelReef level = LevelReef.Reef_L4;
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
        Start = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_START);
        Start.addInputListener(this);
        Select = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.OPERATOR_SELECT);
        Select.addInputListener(this);
        operator_RT = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.OPERATOR_RIGHT_TRIGGER);
        operator_RT.addInputListener(this);
        operator_LT = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.OPERATOR_LEFT_TRIGGER);
        operator_LT.addInputListener(this);
        rightTrigger = (WsJoystickAxis) WsInputs.DRIVER_RIGHT_TRIGGER.get();
        rightTrigger.addInputListener(this);
        operatorDleft = (WsDPadButton) WsInputs.OPERATOR_DPAD_LEFT.get();
        operatorDleft.addInputListener(this);
        operatorDright = (WsDPadButton) WsInputs.OPERATOR_DPAD_RIGHT.get();
        operatorDright.addInputListener(this);
        reset = (WsJoystickButton) WsInputs.OPERATOR_LEFT_JOYSTICK_BUTTON.get();
        reset.addInputListener(this);
        resetAxis = (WsJoystickAxis) WsInputs.OPERATOR_RIGHT_JOYSTICK_Y.get();
        resetAxis.addInputListener(this);
       
        armSpark = (WsSpark) Core.getOutputManager().getOutput(WsOutputs.ARM);
        armSpark.initClosedLoop(0.4, 0, 0, 0);
        armSpark.addClosedLoop(1, 0.2, 0, 0, 0);
        armSpark.setCoast();
        armSpark.setPosition(0);
        armSpark.setCurrentLimit(50, 50, 0);
        LiftMax = (WsSpark) WsOutputs.LIFT.get();
        lift2 = (WsSpark) WsOutputs.LIFT_FOLLOWER.get();
        LiftMax.initClosedLoop(0.2,0,0.1,0);
        lift2.initClosedLoop(0.2,0,0.1,0);
        LiftMax.addClosedLoop(1, 0.05, 0.0, 0.05, 0);
        lift2.addClosedLoop(1, 0.05, 0, 0.05, 0);
        LiftMax.setBrake();
        lift2.setBrake();
        lift2.setPosition(LiftMax.getPosition());
        LiftMax.setCurrentLimit(80, 80, 0);
        lift2.setCurrentLimit(80, 80, 0);

        
    }

@Override
    public void update() {

        AlgaeState = pose.isAlgaeScoreNet() ? Algae_NetOrProces.Net : Algae_NetOrProces.Processor;

        if (isAuto){
            //do nothing
            
        } else if (StartHeld && SelectHeld) {
            desiredPosition = SuperstructurePosition.CLIMB;

        } else if (LShoulderHeld) {
            if (swerve.isCoralStationFront()) {
                desiredPosition = SuperstructurePosition.CORAL_STATION_FRONT;
            } else {
                desiredPosition = SuperstructurePosition.CORAL_STATION_BACK;
            }
        } 
        // else if (RshoulderHeld) {
        //     desiredPosition = SuperstructurePosition.ALGAE_PRESTAGED;
        // } 
        else if (LTHeld) {
            if (swerve.isScoringAlgae()) {
                if (Algae_NetOrProces.Net == AlgaeState) {
                    if (swerve.isNetFront()){
                        if (RTHeld){
                            desiredPosition = SuperstructurePosition.ALGAE_NET_FRONT;
                        } else desiredPosition =  SuperstructurePosition.ALGAE_NET_THROW_FRONT;
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
                // if (coralPath.hasAlgae() || desiredPosition == SuperstructurePosition.STOWED_AFTER_PICKUP_HIGH || 
                //         desiredPosition == SuperstructurePosition.STOWED_AFTER_PICKUP_LOW) {
                //     if (prevPosition == SuperstructurePosition.ALGAE_REEF_LOW) desiredPosition = SuperstructurePosition.STOWED_AFTER_PICKUP_LOW;
                //     else desiredPosition = SuperstructurePosition.STOWED_AFTER_PICKUP_HIGH;
                //     if (armAtPosition()) PickupSequence = false;
                // } else {
                    if (swerve.algaeLow()) {
                        if (dRightHeld || desiredPosition == SuperstructurePosition.ALGAE_REEF_LOW){
                            desiredPosition = SuperstructurePosition.ALGAE_REEF_LOW;
                        } else desiredPosition = SuperstructurePosition.ALGAE_PREPICK_LOW;
                    } else {
                        if (dRightHeld || desiredPosition == SuperstructurePosition.ALGAE_REEF_HIGH){
                            desiredPosition = SuperstructurePosition.ALGAE_REEF_HIGH;
                        } else desiredPosition = SuperstructurePosition.ALGAE_PREPICK_HIGH;
                    }
                    if (coralPath.hasAlgae()) PickupSequence = false;
                // }
            } 
            else {
                // if (LevelReef.Reef_L1 == level) {
                if (isLevel1){
                    //desiredPosition = SuperstructurePosition.CORAL_REEF_L1;
                    desiredPosition = SuperstructurePosition.STOWED;
                } else if (LevelReef.Reef_L2 == level) {
                    desiredPosition = SuperstructurePosition.CORAL_REEF_L2;
                } else if (LevelReef.Reef_L3 == level) {
                    desiredPosition = SuperstructurePosition.CORAL_REEF_L3;
                } else if (LevelReef.Reef_L4 == level) {
                    desiredPosition = SuperstructurePosition.CORAL_REEF_L4;
                }
            }
        } else if (RTHeld){
            if (!isScoreL1()){
                desiredPosition = SuperstructurePosition.GROUND_INTAKE;
            } else desiredPosition = SuperstructurePosition.STOWED;
        } else {
            if (coralPath.hasCoral()){
                if (LevelReef.Reef_L2 == level || PickupSequence) desiredPosition = SuperstructurePosition.STOWED_UP_L2;
                else desiredPosition = SuperstructurePosition.STOWED_UP_L3;
            } else desiredPosition = SuperstructurePosition.STOWED;
        }

        if (prevPosition != desiredPosition && armAtPosition() && liftAtPosition()){
            prevPosition = desiredPosition;
        }

        if (resetStatus){
            armSpark.setSpeed(resetSpeed);
            setLift(0.0);
        } else if (override){
            setLift(SuperstructurePosition.OVERRIDE.getLift());
            setArm(SuperstructurePosition.OVERRIDE.getArm());
        //going to the coral station preset
        // } else if (desiredPosition == SuperstructurePosition.CORAL_STATION_FRONT){

        //     if (armSpark.getPosition() > 11){
        //         setLift(16);
        //     } else setLift(desiredPosition.getLift());
        //     if (LiftMax.getPosition() < 14 && armSpark.getPosition() > 12){
        //         setArm(56);
        //     } else setArm(desiredPosition.getArm());

        //going away from the coral station preset
        } else if (prevPosition == SuperstructurePosition.CORAL_STATION_FRONT){

            if (armSpark.getPosition() < 56){
                setLift(17.3);
            } else setLift(desiredPosition.getLift());
            if (armSpark.getPosition() < 54 && LiftMax.getPosition() < 14){
                setArm(11);
            } else setArm(desiredPosition.getArm());

        //arm wait if scoring on the reef
        } else if (!isAuto && isReefPosition(desiredPosition) && !isReefPosition(prevPosition)){
            setLift(desiredPosition.getLift());
            if (liftAtPosition() || LiftMax.getPosition() > LIFT_STAGE1){
                setArm(desiredPosition.getArm());
            } else {
                setArm(BACK_CLEAR);
            }
        //lift wait if going from the reef
        } else if (!isAuto && !isReefPosition(desiredPosition) && isReefPosition(prevPosition)){
            setArm(desiredPosition.getArm());
            if (isArmClearReef(armSpark.getPosition()) || armAtPosition()){
                setLift(desiredPosition.getLift());
            } else if (prevPosition == SuperstructurePosition.CORAL_REEF_L2 || prevPosition == SuperstructurePosition.CORAL_REEF_L3) {
                setLift(prevPosition.getLift() + 14);
            } else {
                setLift(prevPosition.getLift());
            }
        //normal
        } else {
            setLift(desiredPosition.getLift());
            setArm(desiredPosition.getArm());
        }

        displayNumbers();
}

    @Override
    public void inputUpdate(Input source) {
        resetSpeed = resetAxis.getValue();
        if (reset.getValue()){
            resetStatus = true;
        } else if (resetStatus){
            if (desiredPosition == SuperstructurePosition.CLIMB){
                armSpark.getController().getEncoder().setPosition(22.0);
            } else armSpark.getController().getEncoder().setPosition(0);
            resetStatus = false;
        }
        if (isAuto) isAuto = false;
        LShoulderHeld = LShoulder.getValue();
        LTHeld = Math.abs(LT.getValue()) > 0.5;
        RTHeld = Math.abs(rightTrigger.getValue()) > 0.5;
        dRightHeld = operatorDright.getValue();
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
        if (Rshoulder.getValue()) isLevel1 = true;
        if (Math.abs(rightTrigger.getValue()) > 0.5 && Math.abs(LT.getValue()) < 0.5) isLevel1 = false;
        StartHeld = Start.getValue();
        SelectHeld = Select.getValue();
        if (desiredPosition == SuperstructurePosition.CLIMB){
            StartHeld = true;
            SelectHeld = true;
        }
        override = operatorDleft.getValue();
        
    }
    @Override
    public void initSubsystems() {
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        pose = (WsPose) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_POSE);
    }
    @Override
    public void selfTest() {
        
    }
    @Override
    public void resetState() {
        PickupSequence = false;
        desiredPosition = SuperstructurePosition.STOWED;
        level = LevelReef.Reef_L3;
        AlgaeState = Algae_NetOrProces.Processor;
        dRightHeld = false;
        
    }
    @Override
    public String getName() {
        return "Superstructure Subsystem";
    }
    private void displayNumbers(){
        SmartDashboard.putBoolean("# Algae Pickup", PickupSequence);
        SmartDashboard.putNumber("@ Arm Target", desiredPosition.getArm());
        SmartDashboard.putNumber("@ Arm Position", armSpark.getPosition());
        SmartDashboard.putNumber("@ Lift Target", desiredPosition.getLift());
        SmartDashboard.putNumber("@ Lift Position", LiftMax.getPosition());
        SmartDashboard.putString("@ Superstructure position", desiredPosition.getName());
        SmartDashboard.putString("@ Super prevPosition", prevPosition.getName());
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
        return Math.abs(desiredPosition.getLift()- LiftMax.getPosition())<1.5;
    }
    private boolean armAtPosition(){
        return Math.abs(desiredPosition.getArm() - armSpark.getPosition())<0.5;
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
            desiredPosition == SuperstructurePosition.STOWED_AFTER_PICKUP_LOW ||
            desiredPosition == SuperstructurePosition.ALGAE_REEF_HIGH ||
            desiredPosition == SuperstructurePosition.ALGAE_REEF_LOW ||
            desiredPosition == SuperstructurePosition.ALGAE_PREPICK_LOW ||
            desiredPosition == SuperstructurePosition.ALGAE_PREPICK_HIGH;
    }
    private void setArm(double armPos){
        if (dRightHeld && desiredPosition == SuperstructurePosition.CORAL_REEF_L4){
            armSpark.setPosition(armPos+10, 1);
        } else if (isLiftHigh(LiftMax.getPosition()) || isLiftHigh(desiredPosition)){
            armSpark.setPosition(armPos, 1);
        } else {
            armSpark.setPosition(armPos, 0);
        }
    }
    private void setLift(double liftPos){
        if (liftAtPosition() && desiredPosition.getLift() == 0){
            LiftMax.setPosition(-0.5, 1, LIFT_FF);
            lift2.setPosition(0.5, 1, LIFT_FF);
        }
        if (desiredPosition.getLift() < LiftMax.getPosition() && !isAuto){
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
    private boolean isArmClearReef(double position){
        return position < BACK_CLEAR;
    }
    private boolean isReefPosition(SuperstructurePosition position){
        return position == SuperstructurePosition.CORAL_REEF_L1 || 
            position == SuperstructurePosition.CORAL_REEF_L2 ||
            position == SuperstructurePosition.CORAL_REEF_L3 || 
            position == SuperstructurePosition.CORAL_REEF_L4;
    }
    public boolean isScoreL1() {
        // return level == LevelReef.Reef_L1;
        return isLevel1;
    }
    public boolean isScoreL23(){
        return level == LevelReef.Reef_L2 || level == LevelReef.Reef_L3;
    }
    public void setPosition(SuperstructurePosition position){
        desiredPosition = position;
    }
    public void setToAuto(){
        this.isAuto = true;
    }
    public boolean isOverride(){
        return override;
    }
}
