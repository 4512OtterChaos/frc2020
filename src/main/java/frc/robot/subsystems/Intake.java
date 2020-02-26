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
    private boolean sliderExtended;
    private boolean sliderWantsExtended;
    private boolean lastSliderExtended ;
    private Timer sliderDebounce = new Timer();
    
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    
    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);
    
    private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Positional PID controller
    
    public Intake() {
        OCConfig.configMotors(ConfigType.INTAKEARM, arm);
        OCConfig.configMotors(ConfigType.INTAKE, roller, fence);

        boolean ext = true;
        sliderExtended = ext;
        sliderWantsExtended = ext;
        lastSliderExtended = ext;
        
        roller.setInverted(false);
        fence.setInverted(false);
        arm.setInverted(false);
    }
    
    @Override
    public void periodic() {
        boolean nowSliderExtended;
        // Block the slider if it wants to extend while the arm is in the way
        boolean conflicts = MathHelp.isBetweenBounds(getArmDegrees(), kLowerSafeRotations, kHigherSafeRotations);
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

        boolean armLowered = getArmDegrees()<=kLowerSafeRotations;
        // avoid rolling rollers when arm is up
        rollerVolts = armLowered ? rollerVolts : 0;
        // avoid rolling fences when slider is in or arm is up
        fenceVolts = nowSliderExtended&&armLowered ? fenceVolts : 0;

        // arm safety
        double lowLimit = armVolts;
        double highLimit = armVolts;
        
        // arm safety
        double enc = getArmDegrees();
        boolean ext = getSliderExtended();
        if(ext&&MathHelp.isBetweenBounds(enc, kHigherSafeRotations, kHigherSafeRotations+kBufferRotations)) lowLimit=0; // just in case, anti-collide on slider
        if(ext&&MathHelp.isBetweenBounds(enc, kLowerSafeRotations-kBufferRotations, kLowerSafeRotations)) highLimit=0;
        if(enc<=0) lowLimit=0;
        if(enc>=kMaxUpwardRotations) highLimit=0;
        
        armVolts = MathHelp.clamp(armVolts, lowLimit, highLimit);


        // set mechanisms with safety
        if(nowSliderExtended) slider.set(Value.kForward);
        else slider.set(Value.kReverse);

        arm.setVoltage(armVolts);
        roller.setVoltage(rollerVolts);
        fence.setVoltage(fenceVolts);
    }

    public ProfiledPIDController getController(){
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
        System.out.println("Set slider: "+is);
        sliderWantsExtended = is;
    }
    
    public void setArmVolts(double volts){
    }
    public void setRollerVolts(double volts){
        rollerVolts = volts;
    }
    public void setFenceVolts(double volts){
        fenceVolts = volts;
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
    public void setState(IntakeState state){
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
        SmartDashboard.putBoolean("Slider Extended", getSliderExtended());
        SmartDashboard.putBoolean("Slider Wants", sliderWantsExtended);
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Intake", Status.PASSED);
    }
}
