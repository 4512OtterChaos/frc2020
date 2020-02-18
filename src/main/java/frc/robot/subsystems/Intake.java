/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.IntakeArmConstants.*;
import static frc.robot.common.Constants.*;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable, Testable{

  private CANSparkMax arm;
  private WPI_TalonSRX roller;
  private WPI_TalonSRX fence;

  private DoubleSolenoid slider = new DoubleSolenoid(0, 0);
  private boolean sliderExtended = false;
  private boolean lastSliderExtended = false;
  private Timer sliderDebounce = new Timer();

  private boolean armDownMaxed = false;
  private boolean armUpMaxed = false;
  private final double armUpperBuffer = 0; // use these to avoid hitting slider
  private final double armLowerBuffer = 0;

  private DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);

  @Log
  private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Velocity PID controller
  
  public Intake() {
    super();
    
    arm = OCConfig.createMAX(9, ConfigType.INTAKEARM);
    roller = OCConfig.createSRX(10, ConfigType.INTAKE);
    fence = OCConfig.createSRX(13, ConfigType.INTAKE);

    setArmBrakeMode(true);
    setFenceBrakeMode(true);
    setRollerBrakeMode(true);

    roller.setInverted(false);
    fence.setInverted(false);
    arm.setInverted(true);
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
    double lowLimit = volts;
    double highLimit = volts;

    // arm safety
    double enc = encoder.get();
    if(MathHelp.isBetweenBounds(enc, kHigherSafeRotations, kHigherSafeRotations+kBufferRotations)) lowLimit=0;
    if(MathHelp.isBetweenBounds(enc, kLowerSafeRotations-kBufferRotations, kLowerSafeRotations)) highLimit=0;
    if(enc<=0) lowLimit=0;
    if(enc>=kMaxUpwardRotations) highLimit=0;

    volts = MathHelp.clamp(volts, lowLimit, highLimit);
    arm.setVoltage(volts);
  }
  public void setRollerVolts(double volts){
    roller.setVoltage(volts);
  }
  public void setFenceVolts(double volts){
    fence.setVoltage(volts);
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

  public void setRollerBrakeMode(boolean is){
    roller.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
  }
  public void setFenceBrakeMode(boolean is){
    fence.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
  }
  public void setArmBrakeMode(boolean is){
    arm.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public TestableResult test(){
    return new TestableResult("Intake", Status.PASSED);
  }
}
