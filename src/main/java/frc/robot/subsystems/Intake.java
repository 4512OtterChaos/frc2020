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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.IntakeArmConstants.*;
import static frc.robot.common.Constants.*;

import frc.robot.common.Constants;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.states.IntakeState;
import frc.robot.util.MathHelp;

public class Intake extends SubsystemBase implements Testable{
    
    private CANSparkMax arm = new CANSparkMax(10, MotorType.kBrushless);
    private WPI_TalonSRX roller = new WPI_TalonSRX(11);
    private WPI_TalonSRX fence = new WPI_TalonSRX(12);

    private double armVolts = 0;
    private double rollerVolts = 0;
    private double fenceVolts = 0;
    
    private DoubleSolenoid slider = new DoubleSolenoid(0, 1);
    private boolean sliderExtended = false;
    private boolean sliderWantsExtended = false;
    private boolean lastSliderExtended = false;
    private Timer sliderDebounce = new Timer();
    
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    
    //private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);
    
    //private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Positional PID controller
    private PIDController controller = new PIDController(kP, kI, kD, Constants.kRobotDelta);
    public Intake() {
        OCConfig.configMotors(ConfigType.INTAKEARM, arm);
        OCConfig.configMotors(ConfigType.INTAKE, roller, fence);

        controller.setTolerance(3*kBufferDegrees, 6*kBufferDegrees);
        
        roller.setInverted(false);
        fence.setInverted(false);
        arm.setInverted(true);
    }

    public void init(){
        boolean ext = getNowSliderExtended();
        sliderExtended = ext;
        sliderWantsExtended = ext;
        lastSliderExtended = ext;
    }
    
    @Override
    public void periodic() {
        boolean nowSliderExtended;
        // Block the slider if it wants to extend while the arm is in the way
        boolean conflicts = MathHelp.isBetweenBounds(getArmDegrees(), kLowerSafeDegrees, kHigherSafeDegrees-5*kBufferDegrees);
        if(sliderWantsExtended && !conflicts) nowSliderExtended = true;
        else nowSliderExtended = false;
        
        // Delay the result of the slider if its retracting for safety
        if(nowSliderExtended){
            if(!lastSliderExtended) sliderDebounce.reset();
            sliderExtended=true;
        }
        else{
            if(lastSliderExtended) sliderDebounce.start();
            else if(sliderDebounce.get()>0.4){
                sliderDebounce.stop();
                sliderExtended=false;
            }
        }
        lastSliderExtended=nowSliderExtended;

        boolean armLowered = getArmDegrees()<=kLowerSafeDegrees;
        // avoid rolling rollers when arm is up
        rollerVolts = armLowered ? rollerVolts : 0;
        // avoid rolling fences when slider is in or arm is up
        //fenceVolts = nowSliderExtended&&armLowered ? fenceVolts : 0;

        // arm safety
        double lowLimit = -5;
        double highLimit = 5;
        
        // arm safety
        double enc = getArmDegrees();
        boolean ext = getSliderExtended();
        if(ext&&MathHelp.isBetweenBounds(enc, kHigherSafeDegrees-kBufferDegrees, kHigherSafeDegrees)) lowLimit=0; // just in case, anti-collide on slider
        if(ext&&MathHelp.isBetweenBounds(enc, kLowerSafeDegrees, kLowerSafeDegrees+kBufferDegrees)) highLimit=0;
        if(enc<=kBufferDegrees) highLimit=0;
        if(enc>=kMaxUpwardDegrees) lowLimit=0;
        
        armVolts = MathHelp.clamp(armVolts, lowLimit, highLimit);

        // set mechanisms with safety
        if(nowSliderExtended) slider.set(Value.kForward);
        else slider.set(Value.kReverse);

        double adjustedVolts = armVolts-7*kCounterGravityFF*Math.sin(Units.degreesToRadians(enc))+5*kCounterGravityFF;
        SmartDashboard.putNumber("Arm Volts", armVolts);
        arm.setVoltage(adjustedVolts);
        roller.setVoltage(rollerVolts);
        fence.setVoltage(fenceVolts);
    }

    public PIDController getController(){
        return controller;
    }
    
    public double getEncoder(){
        double rotations = encoder.get() > 0.3 ? encoder.get()-1 : encoder.get();
        return -rotations+kEncoderOffset;
    }
    public double getArmDegrees(){
        return getEncoder()*360;
    }
    
    private boolean getNowSliderExtended(){
        return slider.get() == Value.kForward;
    }
    public boolean getSliderExtended(){
        return sliderExtended;
    }
    
    public void setSliderExtended(boolean is){
        sliderWantsExtended = is;
    }
    
    public void setArmVolts(double volts){
        armVolts = -volts;
    }
    public void setRollerVolts(double volts){
        rollerVolts = volts;
    }
    public void setFenceVolts(double volts){
        fenceVolts = volts;
    }
    /**
    * Sets the controller goal. Automatically adjusts goal to avoid conflicting with the slider.
    * @param degrees Motor degrees setpoint
    */
    public void setArmPID(double degrees){
        // adjust goal to avoid obliterating slider
        double enc = getArmDegrees();
        if(getSliderExtended()){
            boolean constrainUpper; // false: limit lower, true: limit higher
            if(MathHelp.isBetweenBounds(enc, 0, kLowerSafeDegrees)) constrainUpper=false;
            else if(MathHelp.isBetweenBounds(enc, kHigherSafeDegrees, kMaxUpwardDegrees)) constrainUpper=true;
            else{
                double mid = (kHigherSafeDegrees-kLowerSafeDegrees)/2.0;
                constrainUpper = !(enc<=mid); // in case we extend slider while arm is conflicting
            }
            
            if(constrainUpper) degrees = MathHelp.clamp(degrees, kHigherSafeDegrees+kBufferDegrees, kMaxUpwardDegrees-kBufferDegrees);
            else degrees = MathHelp.clamp(degrees, 0, kLowerSafeDegrees-kBufferDegrees);
        }
        
        double volts = controller.calculate(getArmDegrees(), degrees);
        //volts += feedForward.calculate(controller.getGoal().velocity);
        
        setArmVolts(volts);
    }
    public void setState(IntakeState state){
        if(state.angle < kLowerSafeDegrees) setArmBrakeOn(false);
        else setArmBrakeOn(true);
        setArmPID(state.angle);
        setSliderExtended(state.extended);
    }
    
    public void setRollerBrakeOn(boolean is){
        roller.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }
    public void setFenceBrakeOn(boolean is){
        fence.setNeutralMode(is ? NeutralMode.Brake : NeutralMode.Coast);
    }
    public void setArmBrakeOn(boolean is){
        arm.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void log(){
        SmartDashboard.putNumber("Arm Degrees", getArmDegrees());
        SmartDashboard.putNumber("Arm Encoder", getEncoder());
        SmartDashboard.putNumber("Arm Power", arm.getAppliedOutput());
        SmartDashboard.putBoolean("Slider Extended", getSliderExtended());
        SmartDashboard.putBoolean("Slider Wants", sliderWantsExtended);
        SmartDashboard.putBoolean("Slider Extended Now", getNowSliderExtended());
        SmartDashboard.putNumber("Target Arm Position", controller.getSetpoint());
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Intake", Status.PASSED);
    }
}
