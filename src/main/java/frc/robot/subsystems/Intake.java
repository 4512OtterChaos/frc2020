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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.IntakeArmConstants.*;
import static frc.robot.common.Constants.*;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;

public class Intake extends SubsystemBase implements Testable{
    
    private CANSparkMax arm;
    private WPI_TalonSRX roller;
    private WPI_TalonSRX fence;
    
    private DoubleSolenoid slider;
    private boolean sliderWantsExtended = false;
    private boolean sliderExtended = false;
    private boolean lastSliderExtended = false;
    private Timer sliderDebounce = new Timer();
    
    private DutyCycleEncoder encoder;
    
    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);
    
    private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Positional PID controller
    
    public Intake() {
        arm = OCConfig.createMAX(10, ConfigType.INTAKEARM);
        roller = OCConfig.createSRX(11, ConfigType.INTAKE);
        fence = OCConfig.createSRX(12, ConfigType.INTAKE);
        
        slider = new DoubleSolenoid(0, 1);
        
        encoder = new DutyCycleEncoder(0);
        
        roller.setInverted(false);
        fence.setInverted(false);
        arm.setInverted(true);
    }
    
    @Override
    public void periodic() {
        
        // Block the slider if it wants to extend while the arm is in the way
        boolean conflicts = MathHelp.isBetweenBounds(getArmDegrees(), kLowerSafeRotations, kHigherSafeRotations);
        if(sliderWantsExtended){
            if(conflicts) slider.set(Value.kReverse);
            else slider.set(Value.kForward);
        }
        
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
    
    public double getArmDegrees(){
        return (encoder.get()-kEncoderOffset)*360;
    }
    
    public boolean getSliderExtended(){
        return sliderExtended;
    }
    
    public void setSliderExtended(boolean is){
        sliderWantsExtended = is;
    }
    
    public void setArmVolts(double volts){
        double lowLimit = volts;
        double highLimit = volts;
        
        // arm safety
        double enc = getArmDegrees();
        boolean ext = getSliderExtended();
        if(ext&&MathHelp.isBetweenBounds(enc, kHigherSafeRotations, kHigherSafeRotations+kBufferRotations)) lowLimit=0; // just in case, anti-collide on slider
        if(ext&&MathHelp.isBetweenBounds(enc, kLowerSafeRotations-kBufferRotations, kLowerSafeRotations)) highLimit=0;
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
    * Sets the controller goal. Automatically adjusts goal to avoid conflicting with the slider.
    * @param rotations Motor rotations setpoint
    */
    public void setArmPID(double rotations){
        // adjust goal to avoid obliterating slider
        if(getSliderExtended()){
            boolean constrainUpper; // false: limit lower, true: limit higher
            if(MathHelp.isBetweenBounds(rotations, 0, kLowerSafeRotations)) constrainUpper=false;
            else if(MathHelp.isBetweenBounds(rotations, kHigherSafeRotations, kMaxUpwardRotations)) constrainUpper=true;
            else{
                double mid = (kHigherSafeRotations-kLowerSafeRotations)/2.0;
                constrainUpper = !(rotations<=mid); // in case we extend slider while arm is conflicting
            }
            
            if(constrainUpper) rotations = MathHelp.clamp(rotations, kHigherSafeRotations+kBufferRotations, kMaxUpwardRotations-kBufferRotations);
            else rotations = MathHelp.clamp(rotations, 0, kLowerSafeRotations-kBufferRotations);
        }
        
        double volts = controller.calculate(getArmDegrees(), rotations);
        volts += feedForward.calculate(controller.getGoal().velocity);
        
        setArmVolts(volts);
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
        SmartDashboard.putNumber("Arm Encoder", encoder.get());
        SmartDashboard.putNumber("Arm Offset", encoder.getPositionOffset());
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Intake", Status.PASSED);
    }
}
