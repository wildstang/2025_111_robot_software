package org.wildstang.sample.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.Arrays;

import org.wildstang.framework.auto.steps.LambdaStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.swerve.SwerveDriveTemplate;
import org.wildstang.hardware.roborio.outputs.WsSpark;
import org.wildstang.sample.robot.CANConstants;
import org.wildstang.sample.robot.WsInputs;
import org.wildstang.sample.robot.WsOutputs;
import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.CoralPath;
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
    private AnalogInput rightTrigger;//intake, score when aiming
    private AnalogInput leftTrigger;//scoring to speaker
    private DigitalInput rightBumper;//slowdown
    private DigitalInput leftBumper;//lift up to amp
    private DigitalInput select;//gyro reset
    private DigitalInput faceUp;//rotation lock 0 degrees
    private DigitalInput faceRight;//rotation lock 90 degrees
    private DigitalInput faceLeft;//rotation lock 270 degrees
    private DigitalInput faceDown;//rotation lock 180 degrees
    private DigitalInput dpadLeft;//defense mode
    private DigitalInput dpadRight;
    private DigitalInput driverStart; // Auto rotate to reef
    private DigitalInput operatorLeftBumper; // Select left branch auto align
    private DigitalInput operatorRightBumper; // Select right branch auto align
    private AnalogInput operatorLeftTrigger;
    private AnalogInput operatorRightTrigger;
    private DigitalInput operatorDpadUp;
    private DigitalInput operatorDpadLeft;
    private DigitalInput operatorFaceLeft;
    private DigitalInput operatorStart;
    private DigitalInput operatorSelect;

    private double xPower;
    private double yPower;
    private double rotSpeed;
    private boolean rotLocked;

    /**Direction to face */
    private double rotTarget;

    private boolean isReef;
    
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
    private Pose2d targetPose;


    private enum driveType {TELEOP, AUTO, CROSS, REEFSCORE, NETSCORE, PROCESSORSCORE, CORALSTATION, CLIMB};
    private driveType driveState;
    public boolean rightBranch;
    public boolean topTriangle;
    public boolean algaeNet;

    @Override
    public void inputUpdate(Input source) {
        
        // Operator controls set intent state variables
        if (source == operatorLeftBumper && operatorLeftBumper.getValue()) {
            rightBranch = false;
        } else if (source == operatorRightBumper && operatorRightBumper.getValue()) {
            rightBranch = true;
        } else if (source == operatorLeftTrigger && operatorLeftTrigger.getValue() > 0.5) {
            topTriangle = false;
        } else if (source == operatorRightTrigger && operatorRightTrigger.getValue() > 0.5) {
            topTriangle = true;
        } else if (source == operatorDpadUp && operatorDpadUp.getValue()) {
            algaeNet = true;
        } else if (source == operatorDpadLeft && operatorDpadLeft.getValue()) {
            algaeNet = false;
        } else if (source == leftTrigger) {

            if (leftTrigger.getValue() > 0.5) {
                isReef = false;

                // Scoring algae
                if (coralPath.hasAlgae() && !coralPath.hasCoral()) {
                    if (algaeNet) {
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
        }
        if (driveState != driveType.CLIMB && leftTrigger.getValue() < 0.5 && leftBumper.getValue() == false) {
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
        operatorLeftTrigger = (AnalogInput) WsInputs.OPERATOR_LEFT_TRIGGER.get();
        operatorLeftTrigger.addInputListener(this);
        operatorRightTrigger = (AnalogInput) WsInputs.OPERATOR_RIGHT_TRIGGER.get();
        operatorRightTrigger.addInputListener(this);
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
        pose.addOdometryObservation(modulePositions(), odoAngle());

        if (driveState == driveType.CROSS) {
            //set to cross - done in inputupdate
            this.swerveSignal = swerveHelper.setCross();
        } else if (driveState == driveType.TELEOP) {
            if (rotLocked){
                if (isReef){
                    // Oops, the scoring side is on the "back" of the robot now
                    rotTarget = (pose.turnToTarget(VisionConsts.reefCenter)+180)%360;
                } 
                rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                if (WsSwerveHelper.angleDist(rotTarget, getGyroAngle()) < 1.0) rotSpeed = 0;
            }
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
        } else if (driveState == driveType.REEFSCORE) {

            // Automatically p-loop translate to scoring position
            Pose2d targetPose = pose.getClosestBranch(rightBranch, topTriangle);
            rotTarget = targetPose.getRotation().getDegrees();
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            xPower = pose.getAlignX(targetPose.getTranslation());
            yPower = pose.getAlignY(targetPose.getTranslation());
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());

        // Align closest scoring side to 0 and translate to right y position
        } else if (driveState == driveType.NETSCORE) {
            rotTarget = frontCloser(0) ? 0 : 180;
            rotSpeed = swerveHelper.getRotControl(0, getGyroAngle());
            yPower = pose.getAlignY(VisionConsts.netScore);
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());

        // Align closest scoring side to 90
        } else if (driveState == driveType.PROCESSORSCORE) {
            rotTarget = frontCloser(90) ? 90 : 270;
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
        } else if (driveState == driveType.CORALSTATION) {

            // Rotate to whichever coral station is closest
            rotTarget = pose.isClosestStationRight() ? VisionConsts.coralStationRightHeading : VisionConsts.coralStationLeftHeading;
            // Intake on closer side
            if (!frontCloser(rotTarget)) {
                rotTarget = (rotTarget + 180) % 360;
            }
            rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
            // TODO: Set yPower based on LaserCAN
            // Gyro 0 for robot centric X, Y
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, 0);

        // Just heading lock to 90 (climb on left side of robot) so Rossen doesn't accidentally turn
        } else if (driveState == driveType.CLIMB) {
            rotSpeed = swerveHelper.getRotControl(90, getGyroAngle());
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
        // Autonomous period
        } else if (driveState == driveType.AUTO) {
            // TODO: Do something
            rotSpeed = swerveHelper.getAutoRotation((360-targetPose.getRotation().getDegrees())%360, getGyroAngle());
            xPower += pose.getAlignX(targetPose.getTranslation());
            yPower += pose.getAlignY(targetPose.getTranslation());
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
            // Pre generated power values in set auto
        }

        // Rossen tipping???
        if (isRossenTipping()) {
            xPower = gyro.getRoll().getValueAsDouble() * DriveConstants.TIPPING_P;
            yPower = gyro.getPitch().getValueAsDouble() * DriveConstants.TIPPING_P;
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, 0, 0);
        }
        drive();
        SmartDashboard.putNumber("# Robot X", pose.estimatedPose.getX());
        SmartDashboard.putNumber("# Robot Y", pose.estimatedPose.getY());
        SmartDashboard.putNumber("Gyro Reading", getGyroAngle());
        SmartDashboard.putNumber("X Power", xPower);
        SmartDashboard.putNumber("Y Power", yPower);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);
        SmartDashboard.putString("Drive mode", driveState.toString());
        SmartDashboard.putBoolean("rotLocked", rotLocked);
        SmartDashboard.putNumber("Rotation target", rotTarget);
        SmartDashboard.putNumber("Yaw", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Roll", gyro.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Pitch", gyro.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Drive Speed", speedMagnitude());
        SmartDashboard.putBoolean("Alliance Color", DriverStation.getAlliance().isPresent());
        SmartDashboard.putBoolean("@ is blue", Core.isBlue());
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
        // accel of 0 because currently not using acceleration for power since
        xPower = swerveHelper.getAutoPower(xVelocity, 0);
        yPower = swerveHelper.getAutoPower(yVelocity, 0);
        targetPose = target;
    }

    public Pose2d returnPose(){
        return pose.estimatedPose;
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
        return Math.toRadians(gyro.getYaw().getValueAsDouble() + (Core.isBlue() ? 0 : 180));
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
}
