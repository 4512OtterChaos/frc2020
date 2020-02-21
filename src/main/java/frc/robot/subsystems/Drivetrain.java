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
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;

import static frc.robot.common.Constants.kRobotDelta;
import static frc.robot.common.Constants.DrivetrainConstants.*;

import java.util.TreeMap;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable, Testable{
    
    @Log(methodName = "getAppliedOutput")
    private CANSparkMax leftMaster;
    private CANSparkMax leftSlave;
    @Log(methodName = "getAppliedOutput")
    private CANSparkMax rightMaster;
    private CANSparkMax rightSlave;
    
    private final CANSparkMax[] leftMotors = {leftMaster,leftSlave};
    private final CANSparkMax[] rightMotors = {rightMaster,rightSlave};
    
    @Log(methodName = "getVelocity")
    private CANEncoder leftEncoder;
    @Log(methodName = "getVelocity")
    private CANEncoder rightEncoder;

    private final PigeonIMU pigeon;
    private double[] ypr = new double[3]; // yaw, pitch, roll degrees
    private double[] xyz = new double[3]; // x, y, z degrees per second

    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    private DifferentialDriveOdometry odometry;
    private TreeMap<Double, Pose2d> poseHistory = new TreeMap<>();
    private final double poseHistoryWindow = 0.75; // seconds

    @Log
    private PIDController leftController = new PIDController(kP, kI, kD, kRobotDelta); // Velocity PID controllers
    @Log
    private PIDController rightController = new PIDController(kP, kI, kD, kRobotDelta);

    private double driveSpeed = 0.3;
    
    public Drivetrain() {
        super();

        leftMaster = OCConfig.createMAX(3, ConfigType.DRIVE);
        leftSlave = OCConfig.createMAX(4, ConfigType.DRIVE);
        rightMaster = OCConfig.createMAX(1, ConfigType.DRIVE);
        rightSlave = OCConfig.createMAX(2, ConfigType.DRIVE);

        pigeon = new PigeonIMU(0);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        OCConfig.configureDrivetrain(leftMotors, rightMotors, true);
    }
    
    @Override
    public void periodic() {
    }


    //----- Drivetrain control

    public void setDriveSpeed(double speed){
        driveSpeed = speed;
    }

    public void setBrakeOn(boolean is){
        IdleMode mode = is ? IdleMode.kBrake : IdleMode.kCoast;
        OCConfig.setIdleMode(mode, leftMotors);
        OCConfig.setIdleMode(mode, rightMotors);
    }

    /**
     * As opposed to the more basic set() method, using voltage allows for compensation between battery differences.
     * <p>(This method does not actually perform compensation, just turns volts to percentage, and is not affected by drivespeed.
     * See {@link OCConfig})
     * @return double[] outputs (0 left, 1 right)
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        //return tankDrive(leftVolts / 12.0, rightVolts / 12.0, 1.0);
    }
    /**
     * Sets both sides of the drivetrain to given percentages
     * @return double[] outputs (0 left, 1 right)
     */
    public void tankDrive(double left, double right){
        tankDrive(left, right, driveSpeed);
    }
    /**
     * Sets both sides of the drivetrain to given percentages
     * @param driveSpeed Overloads current drivetrain speed
     */
    public void tankDrive(double left, double right, double driveSpeed){
        left *= driveSpeed;
        right *= driveSpeed;
        tankDriveVolts(left*12, right*12);
    }
    /**
     * Feeds percentages into chassis speeds for velocity PID.
     * Positive: linear forward, angular left
     */
    public void setChassisSpeedPID(double linearPercent, double angularPercent){
        double linear = linearPercent*kMaxVelocityMeters;
        double angular = (angularPercent*(kMaxVelocityRadians));
        setChassisSpeedPID(new ChassisSpeeds(linear, 0, angular));
    }
    /**
     * Feeds chassis speeds into differential drive wheel speeds for velocity PID.
     */
    public void setChassisSpeedPID(ChassisSpeeds chassisSpeeds){
        DifferentialDriveWheelSpeeds dSpeeds = getKinematics().toWheelSpeeds(chassisSpeeds);
        setVelocityPID(dSpeeds.leftMetersPerSecond, dSpeeds.rightMetersPerSecond);
    }
    /**
     * Uses PID + FF to achieve given wheel speeds.
     * <p><b>! Note: All inputs are in meters and all outputs are in volts.
     */
    public void setVelocityPID(double leftMetersPerSecond, double rightMetersPerSecond){
        leftMetersPerSecond *= driveSpeed;
        rightMetersPerSecond *= driveSpeed;
        DifferentialDriveWheelSpeeds speeds = getWheelSpeeds();
        double leftVolts = 0;
        double rightVolts = 0;
        
        leftVolts += feedForward.calculate(leftMetersPerSecond);
        rightVolts += feedForward.calculate(rightMetersPerSecond);
        leftVolts += leftController.calculate(speeds.leftMetersPerSecond, leftMetersPerSecond);
        rightVolts += rightController.calculate(speeds.rightMetersPerSecond, rightMetersPerSecond);

        tankDriveVolts(
            leftVolts,
            rightVolts);
    }
    public PIDController getleftController(){
        return leftController;
    }
    public PIDController getrightController(){
        return rightController;
    }
    
    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    public void resetGyro(){
        pigeon.setYaw(0);
    }


    //----- Informational methods

    public PIDController getLeftPIDController(){
        return leftController;
    }
    public PIDController getRightPIDController(){
        return rightController;
    }

    /**
     * @return DifferentialDriveWheelSpeeds object in meters per second
     */
    @Log.ToString()
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        double circumference = Math.PI * 2.0 * kWheelRadiusMeters;
        return new DifferentialDriveWheelSpeeds(
            (leftEncoder.getVelocity() / kGearRatio * circumference) / 60.0,
            (rightEncoder.getVelocity() / kGearRatio * circumference) / 60.0);
    }
    public double getEncoderDistance(CANEncoder encoder){
        return encoder.getPosition() / kGearRatio * (Math.PI * 2.0 * kWheelRadiusMeters);
    }

    public SimpleMotorFeedforward getFeedForward(){
        return feedForward;
    }
    
    public DifferentialDriveKinematics getKinematics(){
        return kinematics;
    }
    
    @Log(methodName = "getDegrees")
    public Rotation2d getHeading(){
        pigeon.getYawPitchRoll(ypr);
        pigeon.getRawGyro(xyz);
        return Rotation2d.fromDegrees(ypr[0]);
    }
    @Log.ToString(methodName = "getPoseMeters")
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
        resetOdometry(new Pose2d(), gyroAngle);
    }
    public void resetOdometry(Pose2d poseMeters, Rotation2d gyroAngle){
        resetEncoders();
        resetGyro();
        odometry.resetPosition(poseMeters, gyroAngle);
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
            if(timestamp-late<=early-timestamp) pastPose = poseHistory.get(late);
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
