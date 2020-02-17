/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.IntakeConstants.*;
import static frc.robot.common.Constants.*;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable, Testable{

  private CANSparkMax arm = OCConfig.createNEO(9, ConfigType.INTAKE);
  private CANSparkMax roller = OCConfig.createNEO(10, ConfigType.INTAKE);
  private WPI_TalonSRX fence = new WPI_TalonSRX(13);

  private DoubleSolenoid slider = new DoubleSolenoid(0, 0);
  private boolean sliderExtended = false;
  private boolean lastSliderExtended = false;
  private Timer sliderDebounce = new Timer();

  private DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);

  @Log
  private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Velocity PID controller
  
  public Intake() {
    arm.setInverted(true);
    fence.setInverted(false);
    fence.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // Delay the result of the slider if its retracting for safety
    boolean nowSliderExtended = slider.get()==Value.kForward;
    if(nowSliderExtended){
      if(!lastSliderExtended) sliderDebounce.reset();
      sliderExtended=true;
    }
    else{
      if(lastSliderExtended) sliderDebounce.start();
      else if(sliderDebounce.get()>0.5){
        sliderDebounce.stop();
        sliderExtended=false;
      }
    }
    lastSliderExtended=nowSliderExtended;
  }

  public boolean getSliderExtended(){
    return sliderExtended;
  }

  public void setArmVolts(double volts){
    if(encoder.get()>=kMaxForwardRotations) volts = Math.max(0, volts);
    if(encoder.get()-kMaxForwardRotations<=0) volts = Math.min(0,volts);
    arm.setVoltage(volts);
  }
  /**
   * Sets the controller goal.
   * @param rotations Motor rotations setpoint
   */
  public void setArmPID(double rotations){
    double volts = controller.calculate(encoder.get(), rotations);
    volts += feedForward.calculate(controller.getGoal().velocity);
    setArmVolts(volts);
  }

  public void setRollerVolts(double volts){
    roller.setVoltage(volts);
  }

  public void setFenceVolts(double volts){
    fence.setVoltage(volts);
  }

  @Override
  public TestableResult test(){
    return new TestableResult("Intake", Status.PASSED);
  }
}
