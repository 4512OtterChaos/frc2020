/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

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
import frc.robot.util.MathHelp;

public class Lift extends SubsystemBase implements Testable{

  private CANSparkMax master;
  private CANSparkMax slave;

  private DoubleSolenoid ratchet;

  private CANEncoder encoder;

  private DigitalInput botSwitch;
  private DigitalInput ratchetSwitch;

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);

  private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Positional PID controller
  
  public Lift() {
    master = OCConfig.createMAX(8, ConfigType.LIFT);
    slave = OCConfig.createMAX(9, ConfigType.LIFT);

    ratchet = new DoubleSolenoid(2, 3);

    encoder = new CANEncoder(master);

    //botSwitch = new DigitalInput(0);
    //ratchetSwitch = new DigitalInput(0);
    
    master.setInverted(false);
    OCConfig.setFollower(master, false, slave);
  }

  @Override
  public void periodic() {
    //if(botSwitch.get()) encoder.setPosition(0);
  }

  //@Log
  public boolean getBotSwitch(){
      return botSwitch.get();
  }
  public boolean getRatchetSwitch(){
      return ratchetSwitch.get();
  }

  public void setVolts(double volts){
    if(botSwitch.get()) volts = Math.max(0,volts);
    if(encoder.getPosition()>=kMaxHeightRotations) volts = Math.min(0,volts);
    if(ratchet.get()==Value.kForward) volts = MathHelp.clamp(volts, -1, 0);
    if(!ratchetSwitch.get()) volts = 0;
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

  public void setBrakeOn(boolean is){
    master.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    slave.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void log(){

  }

  @Override
  public TestableResult test(){
    return new TestableResult("Lift", Status.PASSED);
  }
}
