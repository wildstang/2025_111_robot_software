package org.wildstang.sample.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import java.util.Arrays;

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

    private double xPower;
    private double yPower;
    private double rotSpeed;
    private boolean rotLocked;

    /**Direction to face */
    private double rotTarget;

    private Pose2d targetPose;


    private boolean automaticallyTranslate;
    private boolean automaticallyLockOnReef;
    
    private final double mToIn = 39.37;

    //private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    public final Pigeon2 gyro = new Pigeon2(CANConstants.GYRO);
    public SwerveModule[] modules;
    private SwerveSignal swerveSignal;
    private WsSwerveHelper swerveHelper = new WsSwerveHelper();
    StructArrayPublisher<SwerveModuleState> moduleStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    public ChassisSpeeds speeds;


    private WsPose pose;

    public enum driveType {TELEOP, AUTO, CROSS};
    public driveType driveState;

    @Override
    public void inputUpdate(Input source) {
        if (operatorLeftBumper.getValue()){
            targetPose = pose.getClosestBranch(false);
        }
        if (operatorRightBumper.getValue()){
            targetPose = pose.getClosestBranch(true);
        }
        if (source == leftTrigger) {
            if (leftTrigger.getValue() > 0.5) {
                automaticallyTranslate = true;
            } else {
                automaticallyTranslate = false;
            }
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
        if (source == driverStart) {
            if (driverStart.getValue()) {
                automaticallyLockOnReef = true;
                rotTarget = pose.turnToTarget(VisionConsts.reefCenter);
                rotLocked = true;
            } else {
                automaticallyLockOnReef = false;
            }
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
        if (source == faceUp && faceUp.getValue() && !automaticallyLockOnReef){
            if (faceLeft.getValue()) rotTarget = 315.0;
            else if (faceRight.getValue()) rotTarget = 45.0;
            else  rotTarget = 0.0;
            rotLocked = true;
        }
        if (source == faceLeft && faceLeft.getValue() && !automaticallyLockOnReef){
            if (faceUp.getValue()) rotTarget = 315.0;
            else if (faceDown.getValue()) rotTarget = 225.0;
            else rotTarget = 270.0;
            rotLocked = true;
        }
        if (source == faceDown && faceDown.getValue() && !automaticallyLockOnReef){
            if (faceLeft.getValue()) rotTarget = 225.0;
            else if (faceRight.getValue()) rotTarget = 135.0;
            else rotTarget = 180.0;
            rotLocked = true;
        }
        if (source == faceRight && faceRight.getValue() && !automaticallyLockOnReef){
            if (faceUp.getValue()) rotTarget = 45.0;
            else if (faceDown.getValue()) rotTarget = 135.0;
            else rotTarget = 90.0;
            rotLocked = true;
        }

        //get rotational joystick
        rotSpeed = rightStickX.getValue()*Math.abs(rightStickX.getValue());
        rotSpeed = swerveHelper.scaleDeadband(rotSpeed, DriveConstants.DEADBAND);
        // if (rotSpeed == 0 && rotLocked == false){
        //     if (Math.abs(getGyroAngle() - rotTarget) < 1.0) rotLocked = true;
        //     rotTarget = getGyroAngle();
        // }
        //if the rotational joystick is being used, the robot should not be auto tracking heading
        if (rotSpeed != 0 && !automaticallyLockOnReef) {
            rotLocked = false;
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

        if (automaticallyLockOnReef) {
            rotTarget = pose.turnToTarget(VisionConsts.reefCenter);
        }

        if (driveState == driveType.CROSS) {
            //set to cross - done in inputupdate
            this.swerveSignal = swerveHelper.setCross();
            drive();
        }
        if (driveState == driveType.TELEOP) {
            if (rotLocked){
                rotSpeed = swerveHelper.getRotControl(rotTarget, getGyroAngle());
                if (Math.abs(rotTarget - getGyroAngle()) < 1.0) rotSpeed = 0;
            }
            if (automaticallyTranslate) {
                xPower = pose.getAlignX(targetPose.getTranslation());
                yPower = pose.getAlignY(targetPose.getTranslation());
            }
            this.swerveSignal = swerveHelper.setDrive(xPower, yPower, rotSpeed, getGyroAngle());
            SmartDashboard.putNumber("FR signal", swerveSignal.getSpeed(0));
            drive();
        }
        if (driveState == driveType.AUTO) {
            
            // Pre generated power values in set auto
            drive();        
        } 
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
        short[] shortAcceleration = {0,0,0};
        //gyro.getBiasedAccelerometer(shortAcceleration); I'm not sure what replaces this
        double[] acceleration = new double[3];
        for (int i=0;i<3;i++) {
            acceleration[i] = shortAcceleration[i];
        }
        SmartDashboard.putNumberArray("Accelerometer", acceleration);
        SmartDashboard.putBoolean("Alliance Color", DriverStation.getAlliance().isPresent());
        moduleStatePublisher.set(moduleStates());
    }
    
    @Override
    public void resetState() {
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
    public void setAutoValues(double xVelocity, double yVelocity, double xOffset, double yOffset) {
        SmartDashboard.putNumber("Offset X Power", xOffset * DriveConstants.TRANSLATION_P);
        SmartDashboard.putNumber("Offset Y Power", yOffset * DriveConstants.TRANSLATION_P);
        // accel of 0 because currently not using acceleration for power since
        xPower = swerveHelper.getAutoPower(xVelocity, 0) + (xOffset) * DriveConstants.TRANSLATION_P;
        yPower = swerveHelper.getAutoPower(yVelocity, 0) + (yOffset) * DriveConstants.TRANSLATION_P;
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
    public double getFieldYaw(){
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
}
