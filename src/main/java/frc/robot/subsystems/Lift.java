/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.*;
import static frc.robot.common.Constants.LiftConstants.*;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Lift extends SubsystemBase implements Loggable, Testable{

  @Log(methodName = "getAppliedOutput")
  private CANSparkMax master = OCConfig.createNEO(7, ConfigType.LIFT);
  @Log(methodName = "getAppliedOutput")
  private CANSparkMax slave = OCConfig.createNEO(8, ConfigType.LIFT);

  private DoubleSolenoid ratchet = new DoubleSolenoid(0, 0);

  private CANEncoder encoder = new CANEncoder(master);

  private DigitalInput botSwitch = new DigitalInput(0);
  private DigitalInput ratchetSwitch = new DigitalInput(0);

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);

  @Log
  private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Velocity PID controller
  
  public Lift() {
    OCConfig.setFollower(master, false, slave);
  }

  @Override
  public void periodic() {
    if(botSwitch.get()) encoder.setPosition(0);
  }

  public void setVolts(double volts){
    if(botSwitch.get()) volts = Math.max(0,volts);
    if(encoder.getPosition()>=kMaxHeightRotations) volts = Math.min(0,volts);
    master.setVoltage(volts);
  }

  /**
   * Sets the controller goal.
   * @param rotations Motor rotations setpoint
   */
  public void setPID(double rotations){
    double volts = controller.calculate(encoder.getPosition(), rotations);
    volts += feedForward.calculate(controller.getGoal().velocity);
    setVolts(volts);
  }

  public void setRatchetEngaged(boolean engaged){
    ratchet.set(engaged ? Value.kForward : Value.kReverse);
  }

  @Override
  public TestableResult test(){
    return new TestableResult("Lift", Status.PASSED);
  }
}
