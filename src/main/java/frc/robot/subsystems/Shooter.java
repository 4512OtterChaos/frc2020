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
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable, Testable{

  @Log(methodName = "getAppliedOutput")
  private CANSparkMax shootMaster;
  private CANSparkMax shootSlave;
  private CANSparkMax wrist;

  private CANEncoder shootEncoder;
  private DutyCycleEncoder wristEncoder;

  private SimpleMotorFeedforward shootFF = new SimpleMotorFeedforward(ShooterConstants.kStaticFF, ShooterConstants.kVelocityFF, ShooterConstants.kAccelerationFF);
  private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(ShooterWristConstants.kStaticFF, ShooterWristConstants.kVelocityFF, ShooterWristConstants.kAccelerationFF);

  private CANPIDController shootController;
  @Log
  private ProfiledPIDController wristController =
    new ProfiledPIDController(ShooterWristConstants.kP, ShooterWristConstants.kI, ShooterWristConstants.kD,
      new Constraints(ShooterWristConstants.kVelocityConstraint, ShooterWristConstants.kAccelerationConstraint), kRobotDelta); // Positional PID controller
  
  
  public Shooter() {
    super();

    shootMaster = OCConfig.createMAX(5, ConfigType.SHOOTER);
    shootSlave = OCConfig.createMAX(6, ConfigType.SHOOTER);
    wrist = OCConfig.createMAX(7, ConfigType.SHOOTERWRIST);

    shootEncoder = new CANEncoder(shootMaster);
    wristEncoder = new DutyCycleEncoder(0);

    shootController = new CANPIDController(shootMaster);
    shootController.setP(ShooterConstants.kP);
    shootController.setI(ShooterConstants.kI);
    shootController.setD(ShooterConstants.kD);

    shootMaster.setInverted(false);
    wrist.setInverted(false);
    OCConfig.setFollower(shootMaster, true, shootSlave);
  }

  @Override
  public void periodic() {
  }

  public void setShooterVolts(double volts){
    shootMaster.setVoltage(volts);
  }
  public void setWristVolts(double volts){
    wrist.setVoltage(volts);
  }

  public void setShooterPID(double rpm){
    shootController.setReference(rpm, ControlType.kVelocity, 0, shootFF.calculate(rpm), ArbFFUnits.kVoltage);
  }

  public void setShooterBrakeOn(boolean is){
    shootMaster.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    shootSlave.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
  }
  public void setWristBrakeOn(boolean is){
    wrist.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public TestableResult test(){
    return new TestableResult("Shooter", Status.PASSED);
  }
}
