/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;

import static frc.robot.common.Constants.kRobotDelta;
import static frc.robot.common.Constants.DrivetrainConstants.*;

import java.util.TreeMap;

public class Drivetrain extends SubsystemBase implements Testable{
    
    private CANSparkMax leftMaster = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftSlave = new CANSparkMax(2, MotorType.kBrushless);

    private CANSparkMax rightMaster = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightSlave = new CANSparkMax(4, MotorType.kBrushless);
        
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private PigeonIMU pigeon = new PigeonIMU(13);
    private double[] ypr = new double[3]; // yaw, pitch, roll degrees
    private double[] xyz = new double[3]; // x, y, z degrees per second

    private SimpleMotorFeedforward linearFF = new SimpleMotorFeedforward(kLinearStaticFF, kLinearVelocityFF, kLinearAccelerationFF);
    private SimpleMotorFeedforward angularFF = new SimpleMotorFeedforward(kAngularStaticFF, kAngularVelocityFF, kAngularAccelerationFF);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    private DifferentialDriveOdometry odometry;
    private DifferentialDriveWheelSpeeds lastSpeeds = new DifferentialDriveWheelSpeeds();
    private double lastTime = Timer.getFPGATimestamp();
    private Field2d field = new Field2d();
    private TreeMap<Double, Pose2d> poseHistory = new TreeMap<>();
    private final double poseHistoryWindow = 0.75; // seconds

    private PIDController leftController = new PIDController(kP, kI, kD, kRobotDelta); // Velocity PID controllers
    private PIDController rightController = new PIDController(kP, kI, kD, kRobotDelta);

    private Rotation2d turnToTarget;
    private Rotation2d turnToError;
    
    public Drivetrain() {
        OCConfig.configMotors(ConfigType.DRIVE, leftMaster, leftSlave);
        OCConfig.configMotors(ConfigType.DRIVE, rightMaster, rightSlave);
        
        //OCConfig.configureDrivetrain(new CANSparkMax[]{leftMaster, leftSlave}, new CANSparkMax[]{leftMaster, leftSlave}, false);
        //OCConfig.setStatusFast(leftMaster,leftSlave,rightMaster,rightSlave);
        leftMaster.follow(ExternalFollower.kFollowerDisabled, 0);
        leftSlave.follow(ExternalFollower.kFollowerDisabled, 0);
        rightMaster.follow(ExternalFollower.kFollowerDisabled, 0);
        rightSlave.follow(ExternalFollower.kFollowerDisabled, 0);
        leftMaster.setInverted(false);
        leftMaster.setInverted(false);
        //leftSlave.follow(leftMaster);
        rightMaster.setInverted(true);
        rightSlave.setInverted(true);
        //rightSlave.follow(rightMaster);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        odometry = new DifferentialDriveOdometry(new Rotation2d());
    }
    
    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(odometry.getPoseMeters());
    }

    //----- Drivetrain control

    public void setBrakeOn(boolean is){
        IdleMode mode = is ? IdleMode.kBrake : IdleMode.kCoast;
        OCConfig.setIdleMode(mode, leftMaster, leftSlave, rightMaster, rightSlave);
    }

    /**
     * As opposed to the more basic set() method, using voltage allows for compensation between battery differences.
     * <p>(This method does not actually perform compensation, just turns volts to percentage, and is not affected by drivespeed.
     * See {@link OCConfig})
     * @return double[] outputs (0 left, 1 right)
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        leftMaster.setVoltage(leftVolts);
        leftSlave.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        rightSlave.setVoltage(rightVolts);
    }
    /**
     * Sets both sides of the drivetrain to given percentages
     * (Uses setVoltage())
     */
    public void tankDrive(double left, double right){
        tankDriveVolts(left*12, right*12);
    }
    /**
     * Feeds percentages into chassis speeds for velocity PID.
     * Positive: linear forward, angular counter-clockwise
     */
    public void setChassisSpeed(double linearPercent, double angularPercent){
        linearPercent = MathHelp.clamp(linearPercent, -1, 1);
        //if(angularPercent!=0) angularPercent+=Math.copySign(0.0075, angularPercent);
        angularPercent = MathHelp.clamp(angularPercent, -1, 1);
        double linear = linearPercent*kMaxVelocityMeters;
        double angular = (angularPercent*(kMaxVelocityRadians));
        setChassisSpeed(new ChassisSpeeds(linear, 0, angular));
    }
    /**
     * Feeds chassis speeds into differential drive wheel speeds for velocity PID.
     */
    public void setChassisSpeed(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds dSpeeds = getKinematics().toWheelSpeeds(chassisSpeeds);
        setVelocityMeters(dSpeeds.leftMetersPerSecond, dSpeeds.rightMetersPerSecond);
    }
    /**
     * Uses PID + FF to achieve given wheel speeds.
     * <p><b>! Note: All inputs are in meters and all outputs are in volts.
     */
    public void setVelocityMeters(double leftMetersPerSecond, double rightMetersPerSecond){
        double nowTime = Timer.getFPGATimestamp();
        double dt = nowTime - lastTime;
        DifferentialDriveWheelSpeeds speeds = getWheelSpeeds();
        double leftMetersPerSecondSquared = (speeds.leftMetersPerSecond - lastSpeeds.leftMetersPerSecond)/dt;
        double rightMetersPerSecondSquared = (speeds.rightMetersPerSecond - lastSpeeds.rightMetersPerSecond)/dt;

        double leftVolts = 0;
        double rightVolts = 0;

        SmartDashboard.putNumber("Left DController Actual", Units.metersToFeet(speeds.leftMetersPerSecond));
        SmartDashboard.putNumber("Right DController Actual", Units.metersToFeet(speeds.rightMetersPerSecond));
        SmartDashboard.putNumber("Left DController Target", Units.metersToFeet(leftMetersPerSecond));
        SmartDashboard.putNumber("Right DController Target", Units.metersToFeet(rightMetersPerSecond));
        
        leftVolts += linearFF.calculate(leftMetersPerSecond);
        rightVolts += linearFF.calculate(rightMetersPerSecond);
        leftVolts += leftController.calculate(speeds.leftMetersPerSecond, leftMetersPerSecond);
        rightVolts += rightController.calculate(speeds.rightMetersPerSecond, rightMetersPerSecond);

        tankDriveVolts(
            leftVolts,
            rightVolts
        );
        lastSpeeds = speeds;
        lastTime = nowTime;
    }
    /**
     * Uses percentage of the maximum robot velocity. See {@link Drivetrain#setVelocityMeters(double, double)}
     */
    public void setVelocityPercentage(double leftPercent, double rightPercent){
        setVelocityMeters(leftPercent*kMaxVelocityMeters, rightPercent*kMaxVelocityMeters);
    }
    
    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    public void resetGyro(){
        pigeon.setYaw(0);
    }
    public void setTurnToTarget(double heading){
        turnToTarget = new Rotation2d(Units.degreesToRadians(heading));
    }
    public void setTurnToError(double error){
        turnToError = new Rotation2d(Units.degreesToRadians(error));
    }


    //----- Informational methods

    public PIDController getLeftController(){
        return leftController;
    }
    public PIDController getRightController(){
        return rightController;
    }

    public double getEncoderDistance(CANEncoder encoder){
        return encoder.getPosition() / kGearRatio * (Math.PI * 2.0 * kWheelRadiusMeters);
    }

    /**
     * The linear feed-forward values.
     */
    public SimpleMotorFeedforward getLinearFF(){
        return linearFF;
    }
    /**
     * The angular feed-forward values.
     */
    public SimpleMotorFeedforward getAngularFF(){
        return angularFF;
    }
    
    public DifferentialDriveKinematics getKinematics(){
        return kinematics;
    }
    /**
     * @return DifferentialDriveWheelSpeeds object in meters per second
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        double circumference = Math.PI * 2.0 * kWheelRadiusMeters;
        return new DifferentialDriveWheelSpeeds(
            (leftEncoder.getVelocity() / kGearRatio * circumference) / 60.0,
            (rightEncoder.getVelocity() / kGearRatio * circumference) / 60.0);
    }
    public ChassisSpeeds getChassisSpeed(){
        return getKinematics().toChassisSpeeds(getWheelSpeeds());
    }
    
    public double getYawPosition(){
        pigeon.getYawPitchRoll(ypr);
        pigeon.getRawGyro(xyz);
        return ypr[0];
    }
    public double getContinuousYawPosition(){
        return MathHelp.getContinuousError(getYawPosition(), 360);
    }
    public double getYawVelocity(){
        pigeon.getRawGyro(xyz);
        return xyz[0];
    }
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(getYawPosition());
    }
    public DifferentialDriveOdometry getOdometry(){
        return odometry;
    }
    public void updateOdometry(){
        odometry.update(getHeading(), getEncoderDistance(leftEncoder), getEncoderDistance(rightEncoder));
        updatePoseHistory(odometry.getPoseMeters());
    }
    public void resetOdometry(){
        resetOdometry(getHeading());
    }
    public void resetOdometry(Rotation2d gyroAngle){
        //resetOdometry(new Pose2d(), gyroAngle);
    }
    public void resetOdometry(Pose2d poseMeters){
        resetEncoders();
        resetGyro();
        odometry.resetPosition(poseMeters, getHeading());
    }
    /**
     * Returns a previous recorded pose.
     * @param age Seconds to go back
     */
    public Pose2d getPoseFromHistory(Double age){
        Pose2d pastPose = new Pose2d();
        if(poseHistory.size()>1){
            double now = Timer.getFPGATimestamp();
            double timestamp = now - age;
            double newestTime = poseHistory.lastKey();
            double oldestTime = poseHistory.firstKey();
            timestamp = MathHelp.clamp(timestamp, oldestTime, newestTime);

            Double early = poseHistory.ceilingKey(timestamp);
            Double late = poseHistory.floorKey(timestamp);
            if(timestamp-late<early-timestamp) pastPose = poseHistory.get(late);
            else pastPose = poseHistory.get(early);
        }
        return pastPose;
    }
    private double getPoseHistoryDuration(){
        return poseHistory.lastKey()-poseHistory.firstKey();
    }
    private void updatePoseHistory(Pose2d pose){
        poseHistory.put(Timer.getFPGATimestamp(), pose);
        Double oldestTime = poseHistory.firstKey();
        if(getPoseHistoryDuration()>poseHistoryWindow){ // if the pose history spans more than 0.75 seconds
            poseHistory.remove(oldestTime);
        }
        SmartDashboard.putNumber("1 Second old Yaw", getPoseFromHistory(1.0).getRotation().getDegrees());
    }

    public Rotation2d getTurnToTarget(){
        return turnToTarget;
    }
    public Rotation2d getTurnToError(){
        return turnToError;
    }

    public void log(){
        SmartDashboard.putNumber("Heading", getContinuousYawPosition());
        SmartDashboard.putNumber("Angular Velocity", Units.radiansToDegrees(kinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond));
        SmartDashboard.putNumber("Gyro Velocity", xyz[2]);
        SmartDashboard.putNumber("Linear Velocity", kinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond);
    }

    @Override
    public TestableResult test(){
        resetEncoders();
        resetGyro();
        Timer.delay(1);
        boolean nonZero = (Math.abs(leftEncoder.getPosition()+rightEncoder.getPosition())>2) || (getHeading().getDegrees()>1);
        Status result = nonZero ? Status.WARNING : Status.PASSED;
        return new TestableResult("Drivetrain", result);
    }
    
}
