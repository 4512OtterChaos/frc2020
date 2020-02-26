/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.*;

import java.util.TreeMap;

import frc.robot.common.OCConfig;
import frc.robot.states.ShooterState;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;

public class Shooter extends SubsystemBase implements Testable{

    private CANSparkMax shootLeft = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax shootRight = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax wrist = new CANSparkMax(7, MotorType.kBrushless);

    private double wristVolts = 0;
    
    private CANEncoder leftEncoder = new CANEncoder(shootLeft);
    private CANEncoder rightEncoder = new CANEncoder(shootRight);
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
    
    private SimpleMotorFeedforward leftShootFF = new SimpleMotorFeedforward(ShooterConstants.klStaticFF, ShooterConstants.klVelocityFF, 0);
    private SimpleMotorFeedforward rightShootFF = new SimpleMotorFeedforward(ShooterConstants.krStaticFF, ShooterConstants.krVelocityFF, 0);
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(ShooterWristConstants.kStaticFF, ShooterWristConstants.kVelocityFF, ShooterWristConstants.kAccelerationFF);
    
    private CANPIDController leftController = new CANPIDController(shootLeft);
    private CANPIDController rightController = new CANPIDController(shootRight);
    private ProfiledPIDController wristController =
    new ProfiledPIDController(ShooterWristConstants.kP, ShooterWristConstants.kI, ShooterWristConstants.kD,
    new Constraints(ShooterWristConstants.kVelocityConstraint, ShooterWristConstants.kAccelerationConstraint), kRobotDelta); // Positional PID controller
    
    public Shooter() {
        OCConfig.configMotors(ConfigType.SHOOTER, shootLeft, shootRight);
        OCConfig.configMotors(ConfigType.SHOOTERWRIST, wrist);
        
        leftController.setP(ShooterConstants.kP);
        leftController.setI(ShooterConstants.kI);
        leftController.setD(ShooterConstants.kD);
        rightController.setP(ShooterConstants.kP);
        rightController.setI(ShooterConstants.kI);
        rightController.setD(ShooterConstants.kD);
        
        shootLeft.setInverted(false);
        shootRight.setInverted(true);
        wrist.setInverted(false);
    }
    
    public void periodic() {
        double minVolts = -12;
        double maxVolts = 12;
        final double deg = getWristDegrees();
        final double low = ShooterWristConstants.kClearIntakeRotations;
        final double high = ShooterWristConstants.kHigherSafeRotations;
        final double buffer = ShooterWristConstants.kBufferRotations;
        if(deg<=low+buffer) minVolts = 0;
        if(deg>=high-buffer) maxVolts = 0;
        wristVolts = MathHelp.clamp(wristVolts, minVolts, maxVolts);
        wrist.setVoltage(wristVolts);
    }

    public ProfiledPIDController getWristController(){
        return wristController;
    }

    public double getLeftRPM(){
        return leftEncoder.getVelocity();
    }
    public double getRightRPM(){
        return rightEncoder.getVelocity();
    }

    public double getWristDegrees(){
        return (wristEncoder.get()+ShooterWristConstants.kEncoderOffset)*360;
    }
    
    public void setShooterVolts(double volts){
        shootLeft.setVoltage(volts);
        shootRight.setVoltage(volts);
    }
    public void setWristVolts(double volts){
        wristVolts = volts;
    }
    
    public void setShooterPID(double rpm){
        double rps = rpm / 60.0;
        leftController.setReference(rpm, ControlType.kVelocity, 0, leftShootFF.calculate(rps), ArbFFUnits.kVoltage);
        rightController.setReference(rpm, ControlType.kVelocity, 0, rightShootFF.calculate(rps), ArbFFUnits.kVoltage);
    }
    public void setWristPID(double rotations){
        rotations = MathHelp.clamp(rotations, ShooterWristConstants.kClearIntakeRotations, ShooterWristConstants.kHigherSafeRotations);
        double volts = wristController.calculate(getWristDegrees(), rotations);
        volts += wristFF.calculate(wristController.getGoal().velocity);
        setWristVolts(volts);
    }
    public void setState(ShooterState state){
        setShooterPID(state.rpm);
        setWristPID(state.angle);
    }
    
    public void setShooterBrakeOn(boolean is){
        OCConfig.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast, shootLeft, shootRight);
    }
    public void setWristBrakeOn(boolean is){
        wrist.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void log(){
        SmartDashboard.putNumber("Wrist Degrees", getWristDegrees());
        SmartDashboard.putNumber("Shooter Left RPM", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Right RPM", rightEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Diff", leftEncoder.getVelocity()-rightEncoder.getVelocity());
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Shooter", Status.PASSED);
    }
}
