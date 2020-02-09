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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
import static frc.robot.common.Constants.kRobotDelta;
import static frc.robot.common.Constants.Drivetrain.*;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable{
    
    @Log(methodName = "getAppliedOutput")
    private CANSparkMax leftMaster = new CANSparkMax(4, MotorType.kBrushless), 
    leftSlave = new CANSparkMax(5, MotorType.kBrushless);
    @Log(methodName = "getAppliedOutput")
    private CANSparkMax rightMaster = new CANSparkMax(1, MotorType.kBrushless),
    rightSlave = new CANSparkMax(2, MotorType.kBrushless);
    
    private final CANSparkMax[] leftMotors = {leftMaster,leftSlave};
    private final CANSparkMax[] rightMotors = {rightMaster,rightSlave};
    
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private final PigeonIMU pigeon = new PigeonIMU(0);
    private double[] ypr = new double[3]; // yaw, pitch, roll degrees
    private double[] xyz = new double[3]; // x, y, z degrees per second

    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    private DifferentialDriveOdometry odometry;

    @Log
    private PIDController leftPIDController = new PIDController(kP, kI, kD, kRobotDelta); // Velocity PID controllers
    @Log
    private PIDController rightPIDController = new PIDController(kP, kI, kD, kRobotDelta);

    private double driveSpeed = 0.5;
    
    public Drivetrain() {
        super();

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        OCConfig.configDriveMotors(leftMotors, rightMotors, true);
    }
    
    @Override
    public void periodic() {
    }


    //----- Drivetrain control

    public void setDriveSpeed(double speed){
        driveSpeed = speed;
    }

    public void setIdleMode(IdleMode mode){
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
        leftVolts += leftPIDController.calculate(speeds.leftMetersPerSecond, leftMetersPerSecond);
        rightVolts += rightPIDController.calculate(speeds.rightMetersPerSecond, rightMetersPerSecond);

        tankDriveVolts(
            leftVolts,
            rightVolts);
    }
    public PIDController getLeftPIDController(){
        return leftPIDController;
    }
    public PIDController getRightPIDController(){
        return rightPIDController;
    }
    
    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    public void resetGyro(){
        pigeon.setYaw(0);
    }


    //----- Informational methods

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
}
