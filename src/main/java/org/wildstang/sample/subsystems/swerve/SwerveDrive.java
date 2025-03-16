package org.wildstang.sample.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.Arrays;

import org.wildstang.framework.auto.steps.LambdaStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.inputs.WsJoystickAxis;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.CANConstants;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
import org.wildstang.sample.subsystems.Superstructure.SuperstructureSubsystem;
import org.wildstang.sample.subsystems.targeting.TargetCoordinate;
import org.wildstang.sample.subsystems.targeting.VisionConsts;
import org.wildstang.sample.subsystems.targeting.WsPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**Class: SwerveDrive
 * inputs: driver left joystick x/y, right joystick x, right trigger, right bumper, select, face buttons all, gyro
 * outputs: four swerveModule objects
 * description: controls a swerve drive for four swerveModules through autonomous and teleoperated control
 */
public class SwerveDrive extends SwerveDriveTemplate {
    private AnalogInput leftStickX;//translation joystick x
    private AnalogInput leftStickY;//translation joystick y
    private AnalogInput rightStickX;//rot joystick
    private AnalogInput rightTrigger;//intake, score when scoring sequence
    private AnalogInput leftTrigger;//scoring sequence
    private DigitalInput rightBumper;//prestaged algae intake
    private DigitalInput leftBumper;//hp station pickup
    private DigitalInput select;//gyro reset
    private DigitalInput faceUp;//rotation lock 0 degrees
    private DigitalInput faceRight;//rotation lock 90 degrees
    private DigitalInput faceLeft;//rotation lock 270 degrees
    private DigitalInput faceDown;//rotation lock 180 degrees
    private DigitalInput dpadLeft;
    private DigitalInput dpadRight;
    private DigitalInput driverStart; // Auto rotate to reef
    private DigitalInput operatorLeftBumper; // Select left branch auto align
    private DigitalInput operatorRightBumper; // Select right branch auto align
    private DigitalInput operatorDpadUp;
    private DigitalInput operatorDpadLeft;
    private DigitalInput operatorFaceLeft;
    private DigitalInput operatorStart;
    private DigitalInput operatorSelect;
    private WsJoystickAxis operatorRightTrigger;
    private WsJoystickAxis operatorLeftTrigger;

    private double xPower;
    private double yPower;
    private double rotSpeed;
    private boolean rotLocked;

    public double autonomousScalar = 2.0;

    /**Direction to face */
    private double rotTarget;

    private boolean isReef;
    private boolean scoringAlgae = false;
    
    private final double mToIn = 39.37;

    //private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();
    StructArrayPublisher<SwerveModuleState> moduleStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    public ChassisSpeeds speeds;


    private WsPose pose;
    private CoralPath coralPath;
    private SuperstructureSubsystem superstructure;
    private Pose2d targetPose;
    StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("targetPose", Pose2d.struct).publish();


    private enum driveType {TELEOP, AUTO, CROSS, REEFSCORE, NETSCORE, PROCESSORSCORE, CORALSTATION, CLIMB, CORALINTAKE};
    private driveType driveState;
    public boolean rightBranch;

    @Override
    public void inputUpdate(Input source) {
        if (Math.abs(operatorLeftTrigger.getValue()) > 0.5) scoringAlgae = true;
        if (Math.abs(operatorRightTrigger.getValue()) > 0.5) scoringAlgae = false;
        
        // Operator controls set intent state variables
        if (source == operatorLeftBumper && operatorLeftBumper.getValue()) {
            rightBranch = false;
        } else if (source == operatorRightBumper && operatorRightBumper.getValue()) {
            rightBranch = true;
        } else if (source == leftTrigger) {

            if (Math.abs(leftTrigger.getValue()) > 0.5) {
                isReef = false;

                // Scoring algae
                if ((scoringAlgae && !(coralPath.hasCoral() && !coralPath.hasAlgae())) || (coralPath.hasAlgae() && !coralPath.hasCoral())) {
                    if (pose.isAlgaeScoreNet()) {
                        driveState = driveType.NETSCORE;
                    } else {
                        driveState = driveType.PROCESSORSCORE;
                    }
                 } else {

                    // No matter where we're positioning on the reef to score, we are
                    driveState = driveType.REEFSCORE;
                }
            } 
        } else if (source == leftBumper) {
            if (leftBumper.getValue()) {
                driveState = driveType.CORALSTATION;
            } else {
                if (coralPath.hasCoral()) {
                    isReef = true;
                }
            }

        // If we are only holding down right trigger and now left trigger (for ground intaking) and we have a face button held down then set to intake based on object detection pipeline
        } else if (Math.abs(rightTrigger.getValue()) > 0.5 && !(Math.abs(leftTrigger.getValue()) < 0.5) && ((faceUp.getValue() || faceDown.getValue() || faceLeft.getValue() || faceRight.getValue()))) {
            driveState = driveType.CORALINTAKE;

        // If none of those conditions are met, return to Teleop mode
        } else if (driveState != driveType.CLIMB) {
            driveState = driveType.TELEOP;
        }

        if (operatorStart.getValue() && operatorSelect.getValue()) {
            driveState = driveType.CLIMB;
        }


        //determine if we are in cross or teleop
        // if (driveState != driveType.AUTO && dpadLeft.getValue()) {
        //     driveState = driveType.CROSS;
        //     for (int i = 0; i < modules.length; i++) {
        //         modules[i].setDriveBrake(true);
        //     }
        //     this.swerveSignal = new SwerveSignal(new double[]{0, 0, 0, 0 }, swerveHelper.setCross().getAngles());
        // }
        // else if (driveState == driveType.CROSS || driveState == driveType.AUTO) {
        //     driveState = driveType.TELEOP;
        // }

        // Toggle auto rotate to reef
        if (driverStart.getValue() && source == driverStart) {
            isReef = !isReef;
            rotLocked = isReef;
        }
        if (!leftBumper.getValue() && source == leftBumper && coralPath.hasAlgae()){
            isReef = true;
            rotLocked = isReef;
        }

        if (driveState == driveType.AUTO) driveState = driveType.TELEOP;

        //get x and y speeds
        xPower = swerveHelper.scaleDeadband(leftStickX.getValue(), DriveConstants.DEADBAND);
        yPower = swerveHelper.scaleDeadband(leftStickY.getValue(), DriveConstants.DEADBAND);
        
        
        //reset gyro
        if (source == select && select.getValue()) {
            gyro.setYaw(0.0);
            if (rotLocked) rotTarget = 0.0;
        }

        // Cardinal directions
        if (source == faceUp && faceUp.getValue()){
            if (faceLeft.getValue()) rotTarget = 315.0;
            else if (faceRight.getValue()) rotTarget = 45.0;
            else  rotTarget = 0.0;
            rotLocked = true;
        }
        if (source == faceLeft && faceLeft.getValue()){
            if (faceUp.getValue()) rotTarget = 315.0;
            else if (faceDown.getValue()) rotTarget = 225.0;
            else rotTarget = 270.0;
            rotLocked = true;
        }
        if (source == faceDown && faceDown.getValue()){
            if (faceLeft.getValue()) rotTarget = 225.0;
            else if (faceRight.getValue()) rotTarget = 135.0;
            else rotTarget = 180.0;
            rotLocked = true;
        }
        if (source == faceRight && faceRight.getValue()){
            if (faceUp.getValue()) rotTarget = 45.0;
            else if (faceDown.getValue()) rotTarget = 135.0;
            else rotTarget = 90.0;
            rotLocked = true;
        }
        if (faceDown.getValue() || faceUp.getValue() || faceLeft.getValue() || faceRight.getValue()) isReef = false;

        //get rotational joystick
        rotSpeed = rightStickX.getValue()*Math.abs(rightStickX.getValue());
        rotSpeed = swerveHelper.scaleDeadband(rotSpeed, DriveConstants.DEADBAND);
        // if (rotSpeed == 0 && rotLocked == false){
        //     if (Math.abs(getGyroAngle() - rotTarget) < 1.0) rotLocked = true;
        //     rotTarget = getGyroAngle();
        // }
        //if the rotational joystick is being used, the robot should not be auto tracking heading
        if (rotSpeed != 0) {
            rotLocked = false;
            isReef = false;
        }
        
        //assign thrust - no thrust this year, low cg robot means max speed all the time
        // thrustValue = 1 - DriveConstants.DRIVE_THRUST + DriveConstants.DRIVE_THRUST * Math.abs(rightTrigger.getValue());
        // xSpeed *= thrustValue;
        // ySpeed *= thrustValue;
        // rotSpeed *= thrustValue;

    }
 
    @Override
    public void init() {
        initInputs();
        initOutputs();
        resetState();
        gyro.setYaw(0.0);        
    }

    public void initSubsystems() {
        pose = (WsPose) Core.getSubsystemManager().getSubsystem(WsSubsystems.WS_POSE);
        coralPath = (CoralPath) Core.getSubsystemManager().getSubsystem(WsSubsystems.CORAL_PATH);
        superstructure = (SuperstructureSubsystem) Core.getSubsystemManager().getSubsystem(WsSubsystems.SUPERSTRUCTURE);
    }

    public void initInputs() {
        leftStickX = (AnalogInput) WsInputs.DRIVER_LEFT_JOYSTICK_X.get();
        leftStickX.addInputListener(this);
        leftStickY = (AnalogInput) WsInputs.DRIVER_LEFT_JOYSTICK_Y.get();
        leftStickY.addInputListener(this);
        rightStickX = (AnalogInput) WsInputs.DRIVER_RIGHT_JOYSTICK_X.get();
        rightStickX.addInputListener(this);
        rightTrigger = (AnalogInput) WsInputs.DRIVER_RIGHT_TRIGGER.get();
        rightTrigger.addInputListener(this);
        leftTrigger = (AnalogInput) WsInputs.DRIVER_LEFT_TRIGGER.get();
        leftTrigger.addInputListener(this);
        rightBumper = (DigitalInput) WsInputs.DRIVER_RIGHT_SHOULDER.get();
        rightBumper.addInputListener(this);
        leftBumper = (DigitalInput) WsInputs.DRIVER_LEFT_SHOULDER.get();
        leftBumper.addInputListener(this);
        select = (DigitalInput) WsInputs.DRIVER_SELECT.get();
        select.addInputListener(this);
        faceUp = (DigitalInput) WsInputs.DRIVER_FACE_UP.get();
        faceUp.addInputListener(this);
        faceLeft = (DigitalInput) WsInputs.DRIVER_FACE_LEFT.get();
        faceLeft.addInputListener(this);
        faceRight = (DigitalInput) WsInputs.DRIVER_FACE_RIGHT.get();
        faceRight.addInputListener(this);
        faceDown = (DigitalInput) WsInputs.DRIVER_FACE_DOWN.get();
        faceDown.addInputListener(this);
        dpadLeft = (DigitalInput) WsInputs.DRIVER_DPAD_LEFT.get();
        dpadLeft.addInputListener(this);
        dpadRight = (DigitalInput) WsInputs.DRIVER_DPAD_RIGHT.get();
        dpadRight.addInputListener(this);
        driverStart = (DigitalInput) WsInputs.DRIVER_START.get();
        driverStart.addInputListener(this);
        operatorLeftBumper = (DigitalInput) WsInputs.OPERATOR_LEFT_SHOULDER.get();
        operatorLeftBumper.addInputListener(this);
        operatorRightBumper = (DigitalInput) WsInputs.OPERATOR_RIGHT_SHOULDER.get();
        operatorRightBumper.addInputListener(this);
        operatorDpadUp = (DigitalInput) WsInputs.OPERATOR_DPAD_UP.get();
        operatorDpadUp.addInputListener(this);
        operatorDpadLeft = (DigitalInput) WsInputs.OPERATOR_DPAD_LEFT.get();
        operatorDpadLeft.addInputListener(this);
        operatorFaceLeft = (DigitalInput) WsInputs.OPERATOR_FACE_LEFT.get();
        operatorFaceLeft.addInputListener(this);
        operatorSelect = (DigitalInput) WsInputs.OPERATOR_SELECT.get();
        operatorSelect.addInputListener(this);
        operatorStart = (DigitalInput) WsInputs.OPERATOR_START.get();
        operatorStart.addInputListener(this);
        operatorLeftTrigger = (WsJoystickAxis) WsInputs.OPERATOR_LEFT_TRIGGER.get();
        operatorLeftTrigger.addInputListener(this);
        operatorRightTrigger = (WsJoystickAxis) WsInputs.OPERATOR_RIGHT_TRIGGER.get();
        operatorRightTrigger.addInputListener(this);
    }

    public void initOutputs() {
        

        //create four swerve modules
        modules = new SwerveModule[]{
            new SwerveModule((WsSpark) WsOutputs.DRIVE1.get(), 
                (WsSpark) WsOutputs.ANGLE1.get(), DriveConstants.FRONT_LEFT_OFFSET),
            new SwerveModule((WsSpark) WsOutputs.DRIVE2.get(), 
                (WsSpark) WsOutputs.ANGLE2.get(), DriveConstants.FRONT_RIGHT_OFFSET),
            new SwerveModule((WsSpark) WsOutputs.DRIVE3.get(), 
                (WsSpark) WsOutputs.ANGLE3.get(), DriveConstants.REAR_LEFT_OFFSET),
            new SwerveModule((WsSpark) WsOutputs.DRIVE4.get(), 
                (WsSpark) WsOutputs.ANGLE4.get(), DriveConstants.REAR_RIGHT_OFFSET)
        };
        //create default swerveSignal
        swerveSignal = new SwerveSignal(new double[]{0.0, 0.0, 0.0, 0.0}, new double[]{0.0, 0.0, 0.0, 0.0});
    }
    
    @Override
    public void selfTest() {
    }

    @Override
    public void update() {
        pose.addOdometryObservation(modulePositions(), odoAngle(), driveState == driveType.AUTO);

        if (driveState == driveType.CROSS) {
            //set to cross - done in inputupdate
            this.swerveSignal = swerveHelper.setCross();
        } else if (driveState == driveType.TELEOP) {
            if (rotLocked){
                if (isReef){
                    // Oops, the scoring side is on the "back" of the robot now
                    rotTarget = (pose.turnToTarget(VisionConsts.reefCenter)+180)%360;
                    rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle(), 1.50);
                } else {
                    rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                }
                if (WsSwerveHelper.angleDist(rotTarget, getGyroAngle()) < 1.0) rotSpeed = 0;
            }
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());

        // If we want to use object detection pipeline to align to coral
        // Aligns heading to face coral and then p-loop to intake it
        // If no coral seen, or we already have a coral. Then normal teleop behavior
        } else if (driveState == driveType.CORALINTAKE) {
            if (pose.getCoralPose().isPresent() && !coralPath.hasCoral()) {
                Translation2d coral = pose.getCoralPose().get();
                rotTarget = pose.turnToTarget(coral);
                rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                if (WsSwerveHelper.angleDist(rotTarget, getGyroAngle()) < 20.0) {
                    xPower = pose.getAlignX(coral);
                    yPower = pose.getAlignY(coral);
                }
                this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
            } else {
                this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
            }
            


        } else if (driveState == driveType.REEFSCORE) {

            // Automatically p-loop translate to scoring position
            if (superstructure.isAlgaeRemoval()) {
                targetPose = pose.getClosestBranch(false);
            } else {
                targetPose = superstructure.isScoreL1() ? pose.getClosestL1Branch(rightBranch) : pose.getClosestBranch(rightBranch);
            }
            rotTarget = targetPose.getRotation().getDegrees();
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            xPower = xPower * 0.5 + pose.getAlignX(targetPose.getTranslation());
            yPower = yPower * 0.5 + pose.getAlignY(targetPose.getTranslation());
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());

        // Align closest scoring side to 0 and translate to right y position
        } else if (driveState == driveType.NETSCORE) {
            rotTarget = frontCloser(0) ? 0 : 180;
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            yPower = pose.getAlignY(VisionConsts.netScore);
            this.swerveSignal = swerveHelper.setDrive(xPower*0.6, yPower, rotSpeed, getGyroAngle());

        // Align closest scoring side to 90
        } else if (driveState == driveType.PROCESSORSCORE) {
            rotTarget = frontCloser(90) ? 90 : 270;
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
        } else if (driveState == driveType.CORALSTATION) {

            // Rotate to whichever coral station is closest
            rotTarget = pose.isClosestStationRight() ? VisionConsts.coralStationRightHeading : VisionConsts.coralStationLeftHeading;
            // Intake on closer side
            if (!frontCloser(rotTarget) || coralPath.hasAlgae()) {
                rotTarget = (rotTarget + 180) % 360;
            }
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            // Gyro 0 for robot centric X, Y
            this.swerveSignal = swerveHelper.setDrive(0.75*xPower, 0.75*yPower, rotSpeed, getGyroAngle());

        // Just heading lock to 90 (climb on left side of robot) so Rossen doesn't accidentally turn
        } else if (driveState == driveType.CLIMB) {
            //rotSpeed = swerveHelper.getRotControl(90, getGyroAngle());
            if (rotLocked){
                rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            }
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
        // Autonomous period
        } else if (driveState == driveType.AUTO) {

            xPower += pose.getAlignX(targetPose.getTranslation());
            yPower += pose.getAlignY(targetPose.getTranslation());
            if (Math.hypot(xPower,yPower) > autonomousScalar){
                double temphypot = Math.hypot(xPower, yPower);
                xPower *= (autonomousScalar / temphypot);
                yPower *= (autonomousScalar / temphypot);
            }
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
            
            SmartDashboard.putNumber("X Power", xPower);
            SmartDashboard.putNumber("Y Power", yPower);
            xPower = 0;
            yPower = 0;
            // Pre generated power values in set auto
        } else {
            
        SmartDashboard.putNumber("X Power", xPower);
        SmartDashboard.putNumber("Y Power", yPower);
        }

        // Rossen tipping???
        // if (isRossenTipping()) {
        //     xPower = gyro.getRoll().getValueAsDouble() * DriveConstants.TIPPING_P;
        //     yPower = gyro.getPitch().getValueAsDouble() * DriveConstants.TIPPING_P;
        //     this.swerveSignal = swerveHelper.setDrive(xPower, yPower, 0, 0);
        // }
        drive();
        SmartDashboard.putNumber("# Robot X", pose.estimatedPose.getX());
        SmartDashboard.putNumber("# Robot Y", pose.estimatedPose.getY());
        SmartDashboard.putNumber("Gyro Reading", getGyroAngle());
        SmartDashboard.putNumber("rotSpeed", rotSpeed);
        SmartDashboard.putString("Drive mode", driveState.toString());
        SmartDashboard.putBoolean("rotLocked", rotLocked);
        SmartDashboard.putNumber("Rotation target", rotTarget);
        SmartDashboard.putNumber("Yaw", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Roll", gyro.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Pitch", gyro.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("@ mega2 gyro", getMegaTag2Yaw());
        SmartDashboard.putNumber("@ speed", speedMagnitude());
        SmartDashboard.putBoolean("# right branch", rightBranch);
        SmartDashboard.putBoolean("# left branch", !rightBranch);
        SmartDashboard.putBoolean("# scoring element", scoringAlgae);
        if (targetPose != null){
            targetPosePublisher.set(targetPose);
        }
        moduleStatePublisher.set(moduleStates());
    }
    
    @Override
    public void resetState() {
        isReef = false;
        xPower = 0;
        yPower = 0;
        rotSpeed = 0;
        rotLocked = false;
        rotTarget = 0.0;
        setToTeleop();
    }

    @Override
    public String getName() {
        return "Swerve Drive";
    }

    /** sets the drive to teleop/cross, and sets drive motors to coast */
    public void setToTeleop() {
        driveState = driveType.TELEOP;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(true);
        }
        rotSpeed = 0;
        xPower = 0;
        yPower = 0;
        rotLocked = false;
    }

    /**sets the drive to autonomous */
    public void setToAuto() {
        driveState = driveType.AUTO;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrake(true);
        }
    }

    /**drives the robot at the current swerveSignal, and displays information for each swerve module */
    private void drive() {
        if (driveState == driveType.CROSS) {
            for (int i = 0; i < modules.length; i++) {
                modules[i].runCross(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
                modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
            }
        }
        else {
            for (int i = 0; i < modules.length; i++) {
                modules[i].run(swerveSignal.getSpeed(i), swerveSignal.getAngle(i));
                modules[i].displayNumbers(DriveConstants.POD_NAMES[i]);
            }
        }
    }



    /**sets autonomous values from the path data file in field relative */
    public void setAutoValues(double xVelocity, double yVelocity, Pose2d target) {
        SmartDashboard.putNumber("Path Velocity", Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)));
        // accel of 0 because currently not using acceleration for power since
        xPower = swerveHelper.getAutoPower(xVelocity, 0);
        yPower = swerveHelper.getAutoPower(yVelocity, 0);
        targetPose = target;
    }

    /**sets the autonomous heading controller to a new target */
    public void setAutoHeading(double headingTarget) {
        rotTarget = headingTarget;
    }

    /**
     * Resets the gyro, and sets it the input number of degrees
     * Used for starting the match at a non-0 angle
     * @param degrees the current value the gyro should read
     */
    public void setGyro(double degrees) {
        resetState();
        setToAuto();

        // Make degrees clockwise
        gyro.setYaw((360-degrees)%360);
    }

    public double getGyroAngle() {
        return (360 - gyro.getYaw().getValueAsDouble()+360)%360;
    }  

    /**
     * @return Returns the field relative CCW gyro angle for use with Limelight MegaTag2
     */
    public double getMegaTag2Yaw(){
        return (gyro.getYaw().getValueAsDouble() + (Core.isBlue() ? 0 : 180));
    }
    /** 
     * @return Returns alliance relative CCW gyro angle for use with always alliance relative pose
     */
    public Rotation2d odoAngle(){
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }
    public double speedMagnitude(){
        return Math.sqrt(Math.pow(speeds().vxMetersPerSecond, 2) + Math.pow(speeds().vyMetersPerSecond, 2));
    }

    private ChassisSpeeds speeds() {
        return DriveConstants.kinematics.toChassisSpeeds(new SwerveModuleState[]
        {modules[0].moduleState(), modules[1].moduleState(), modules[2].moduleState(), modules[3].moduleState()});
    }
    public SwerveModulePosition[] modulePositions(){
        return new SwerveModulePosition[]{modules[0].odoPosition(), modules[1].odoPosition(), modules[2].odoPosition(), modules[3].odoPosition()};
    }
    public SwerveModuleState[] moduleStates() {
        return new SwerveModuleState[]{modules[0].moduleState(), modules[1].moduleState(), modules[2].moduleState(), modules[3].moduleState()};
    }

    private boolean frontCloser(double targetAngle) {
        return (WsSwerveHelper.angleDist(targetAngle, getGyroAngle()) < 90);
    }

    // SUBSYSTEM ACCESS METHODS

    public boolean isRossenTipping() {
        return (WsSwerveHelper.angleDist(gyro.getRoll().getValueAsDouble(), 0) > 10) || (WsSwerveHelper.angleDist(gyro.getPitch().getValueAsDouble(), 0)) > 10;
    }
    public boolean isCoralStationFront(){
        if (coralPath.hasAlgae()) return false;
        return frontCloser(pose.isClosestStationRight() ? VisionConsts.coralStationRightHeading : VisionConsts.coralStationLeftHeading);
    }
    public boolean isProcessorFront(){
        return frontCloser(90);
    }
    public boolean isNetFront(){
        return frontCloser(0);
    }
    public boolean isAtPosition() {
        return pose.estimatedPose.getTranslation().getDistance(targetPose.getTranslation()) < DriveConstants.POSITION_TOLERANCE;
    }
    public boolean isNearReef(){
        return pose.nearReef();
    }
    public boolean algaeLow(){
        //return rotTarget == 0 || rotTarget == 120 || rotTarget == 240;
        return pose.currentID == 6 || pose.currentID == 8 || pose.currentID == 10
             || pose.currentID == 17 || pose.currentID == 19 || pose.currentID == 21;
    }
    public boolean isScoringAlgae(){
        return scoringAlgae;
    }
    public boolean isScoringCoral(){
        return driveState == driveType.REEFSCORE;
    }
    public void setAutoScalar(double scalar){
        this.autonomousScalar = scalar;
    }
}
