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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.*;

import java.util.TreeMap;

import frc.robot.common.OCConfig;
import frc.robot.common.ShooterState;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable, Testable{

    TreeMap<Double, ShooterState> shooterMap = new TreeMap<Double, ShooterState>();
    
    @Log(methodName = "getAppliedOutput")
    private CANSparkMax shootLeft;
    @Log(methodName = "getAppliedOutput")
    private CANSparkMax shootRight;
    @Log(methodName = "getAppliedOutput")
    private CANSparkMax wrist;
    
    @Log(methodName = "getVelocity")
    private CANEncoder leftEncoder;
    @Log(methodName = "getVelocity")
    private CANEncoder rightEncoder;
    private DutyCycleEncoder wristEncoder;
    
    private SimpleMotorFeedforward shootFF = new SimpleMotorFeedforward(ShooterConstants.kStaticFF, ShooterConstants.kVelocityFF, ShooterConstants.kAccelerationFF);
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(ShooterWristConstants.kStaticFF, ShooterWristConstants.kVelocityFF, ShooterWristConstants.kAccelerationFF);
    
    private CANPIDController leftController;
    private CANPIDController rightController;
    @Log
    private ProfiledPIDController wristController =
    new ProfiledPIDController(ShooterWristConstants.kP, ShooterWristConstants.kI, ShooterWristConstants.kD,
    new Constraints(ShooterWristConstants.kVelocityConstraint, ShooterWristConstants.kAccelerationConstraint), kRobotDelta); // Positional PID controller
    
    public Shooter() {
        shooterMap.put(120.0, new ShooterState(30, 4000));
        
        shootLeft = OCConfig.createMAX(5, ConfigType.SHOOTER);
        shootRight = OCConfig.createMAX(6, ConfigType.SHOOTER);
        wrist = OCConfig.createMAX(7, ConfigType.SHOOTERWRIST);
        
        leftEncoder = new CANEncoder(shootLeft);
        rightEncoder = new CANEncoder(shootRight);
        wristEncoder = new DutyCycleEncoder(0);
        
        leftController = new CANPIDController(shootLeft);
        rightController = new CANPIDController(shootRight);
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
    
    @Override
    public void periodic() {
    }

    @Log
    public double getWristEncoder(){
        return -wristEncoder.get();
    }
    
    public void setShooterVolts(double volts){
        shootLeft.setVoltage(volts);
        shootRight.setVoltage(volts);
    }
    public void setWristVolts(double volts){
        wrist.setVoltage(volts);
    }
    
    public void setShooterPID(double rpm){
        leftController.setReference(rpm, ControlType.kVelocity, 0, shootFF.calculate(rpm), ArbFFUnits.kVoltage);
        rightController.setReference(rpm, ControlType.kVelocity, 0, shootFF.calculate(rpm), ArbFFUnits.kVoltage);
    }
    public void setWristPID(double rotations){
        rotations = MathHelp.clamp(rotations, ShooterWristConstants.kLowerSafeRotations, ShooterWristConstants.kHigherSafeRotations);
        double volts = wristController.calculate(getWristEncoder(), rotations);
        volts += wristFF.calculate(wristController.getGoal().velocity);
        setWristVolts(volts);
    }
    
    public void setShooterBrakeOn(boolean is){
        shootLeft.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
        shootRight.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }
    public void setWristBrakeOn(boolean is){
        wrist.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Shooter", Status.PASSED);
    }
}