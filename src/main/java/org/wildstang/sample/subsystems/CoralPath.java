package org.wildstang.sample.subsystems;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.roborio.inputs.WsDPadButton;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.hardware.roborio.inputs.WsJoystickButton;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.CANConstants;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralPath implements Subsystem{


    private SuperstructureSubsystem superstructure;
    private SwerveDrive swerve;

    private WsSpark algae;
    private WsSpark coral;

    private WsJoystickButton leftShoulder;
    private WsJoystickButton rightShoulder;
    private WsJoystickAxis leftTrigger;
    private WsJoystickAxis rightTrigger;
    private WsJoystickAxis operatorLeftTrigger;
    private WsJoystickAxis operatorRightTrigger;
    public WsLaserCAN algaeLC = new WsLaserCAN(CANConstants.ALGAE_LASERCAN, 36);
    public WsLaserCAN coralLC = new WsLaserCAN(CANConstants.CORAL_LASERCAN, 20);

    private double algaeSpeed;
    private double coralSpeed;
    public enum IntakeState { NEUTRAL, INTAKING, SCORING }
    private IntakeState coralState = IntakeState.NEUTRAL;
    private IntakeState algaeState = IntakeState.NEUTRAL;


    @Override
    public void inputUpdate(Input source) {

        
        if (Math.abs(rightTrigger.getValue()) > 0.5) {
            if (Math.abs(leftTrigger.getValue()) > 0.5) {
                if (swerve.isScoringCoral()) {
                    if (!superstructure.isAlgaeRemoval()){
                        coralState = IntakeState.SCORING;
                    }
                } else {
                    algaeState = IntakeState.SCORING;
                }
            } else {
                coralState = IntakeState.INTAKING;
            } 
        } else if (leftTrigger.getValue() > 0.5 && superstructure.isAlgaeRemoval()) {
            algaeState = IntakeState.INTAKING;
        } else {
            coralState = IntakeState.NEUTRAL;
            algaeState = IntakeState.NEUTRAL;
        }
        
        if (source == leftShoulder) {
            coralState = leftShoulder.getValue() ? IntakeState.INTAKING : IntakeState.NEUTRAL;
        }
    }

    @Override
    public void init() {
        algae = (WsSpark) WsOutputs.ALGAE_INTAKE.get();
        coral = (WsSpark) WsOutputs.CORAL_INTAKE.get();

        coral.setBrake();
        coral.setCurrentLimit(60,60,0);//60 for med wheels
        algae.setBrake();
        algae.setCurrentLimit(60,60,0);

        leftShoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_SHOULDER);
        leftShoulder.addInputListener(this);
        rightShoulder = (WsJoystickButton) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_SHOULDER);
        rightShoulder.addInputListener(this);        
        rightTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_RIGHT_TRIGGER);
        rightTrigger.addInputListener(this);
        leftTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.DRIVER_LEFT_TRIGGER);
        leftTrigger.addInputListener(this);
        operatorLeftTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.OPERATOR_LEFT_TRIGGER);
        operatorLeftTrigger.addInputListener(this);
        operatorRightTrigger = (WsJoystickAxis) Core.getInputManager().getInput(WsInputs.OPERATOR_RIGHT_TRIGGER);
        operatorRightTrigger.addInputListener(this);
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        algaeLC.updateMeasurements();
        coralLC.updateMeasurements();
        
        switch (coralState) {
            case INTAKING:
                coralSpeed = 1.0;
                if (hasCoral()) coralSpeed = 0.05;
                break;
            case SCORING:
                if (superstructure.isScoreL23()) coralSpeed = -0.7;//-0.6 for med wheels
                else coralSpeed = -1.0;
                break;
            case NEUTRAL:
                if (hasCoral()){
                    coralSpeed = 0.1;
                } else {
                    coralSpeed = 0.0;
                }
                break;
        }
        switch (algaeState) {
            case INTAKING:
                algaeSpeed = 1.0;
                if (hasAlgae()) algaeState = IntakeState.NEUTRAL;
                break;
            case SCORING:
                algaeSpeed = swerve.isNetFront() ? -0.5 : -1.0;
                break;
            case NEUTRAL:
                if (algaeLC.blocked(25)) {
                    // Stall current
                    algaeSpeed = 0.6;
                } else if (algaeLC.blocked(35)){
                    algaeSpeed = 0.8;
                } else if (algaeLC.blocked(50)){
                    algaeSpeed = 1.0;
                } else {
                    algaeSpeed = 0.0;
                }
                break;
        }
        
        coral.setSpeed(coralSpeed);
        algae.setSpeed(algaeSpeed);
        displayNumbers();
    }

    @Override
    public void resetState() {
    }

    @Override
    public void initSubsystems() {
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
    }

    private void displayNumbers(){
        SmartDashboard.putBoolean("# Has Coral", hasCoral());
        SmartDashboard.putBoolean("# Has Algae", hasAlgae());
        SmartDashboard.putNumber("@ Coral Speed", coralSpeed);
        SmartDashboard.putNumber("@ Algae Speed", algaeSpeed);
        SmartDashboard.putNumber("@ Coral Current", coral.getController().getOutputCurrent());
        SmartDashboard.putNumber("@ Algae Current", algae.getController().getOutputCurrent());
        SmartDashboard.putString("@ Algae State", algaeState.toString());
        SmartDashboard.putString("@ Coral State", coralState.toString());
        algaeLC.putData();
        coralLC.putData();
    }

    public boolean hasCoral() {
        return coralLC.blocked() || superstructure.isScoringCoral();
    }

    public boolean hasAlgae() {
        return algaeLC.blocked() || superstructure.isScoringAlgae();
    }

    public void setIntake(IntakeState state) {
        coralState = state;
    }
    public void getAlgae(){
        algaeState = IntakeState.INTAKING;
    }

    @Override
    public String getName() {
        return "CoralPath";
    }
}
