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
import static frc.robot.common.Constants.IntakeConstants.*;
import static frc.robot.common.Constants.*;

import frc.robot.common.Constants;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;

public class Intake extends SubsystemBase implements Testable{
    
    /*
    private CANSparkMax arm = new CANSparkMax(10, MotorType.kBrushless);
    private WPI_TalonSRX roller = new WPI_TalonSRX(11);
    private WPI_TalonSRX fence = new WPI_TalonSRX(12);
    */
    private CANSparkMax roller = new CANSparkMax(10, MotorType.kBrushless);
    private CANSparkMax leftFence = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax rightFence = new CANSparkMax(12, MotorType.kBrushless);

    private double rollerVolts = 0;
    private double fenceVolts = 0;
    
    private DoubleSolenoid arm = new DoubleSolenoid(0, 1);
    private DoubleSolenoid slider = new DoubleSolenoid(2, 3);
    private boolean armWantsExtended = false;
    private boolean sliderWantsExtended = true;
    
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);
        
    public Intake() {
        OCConfig.configMotors(ConfigType.INTAKEROLLER, roller);
        OCConfig.configMotors(ConfigType.INTAKESLIDE, leftFence, rightFence);
        
        roller.setInverted(false);
        leftFence.setInverted(true);
        rightFence.setInverted(false);
    }

    public void init(){
        armWantsExtended = getArmIsExtended();
    }
    
    @Override
    public void periodic() {
        // arm is "down"
        boolean armLowered = getArmIsLowered();
        // arm is extended
        boolean armExt = getArmIsExtended();
        // slider is extended
        boolean sliderExt = getSliderIsExtended();

        // avoid rolling rollers when arm is up
        rollerVolts = armLowered ? rollerVolts : 0;

        // do NOT move arm if slider isn't extended
        if(sliderExt){
            arm.set(armWantsExtended ? Value.kReverse : Value.kForward);
        }

        slider.set(sliderWantsExtended ? Value.kReverse : Value.kForward);
                
        roller.setVoltage(-rollerVolts);
        leftFence.setVoltage(fenceVolts);
        rightFence.setVoltage(fenceVolts);
    }
    
    public double getEncoder(){
        double rotations = encoder.get() > 0.3 ? encoder.get()-1 : encoder.get();
        return -rotations+kEncoderOffset;
    }
    public double getArmDegrees(){
        return getEncoder()*360;
    }
    public boolean getArmIsLowered(){
        return getArmDegrees()<=kEngagedDegrees;
    }
    public boolean getArmIsExtended(){
        return arm.get() == Value.kReverse;
    }
    public boolean getSliderIsExtended(){
        return slider.get() == Value.kReverse;
    }
    
    public void setArmIsExtended(boolean is){
        armWantsExtended = is;
    }
    public void setSliderIsExtended(boolean is){
        sliderWantsExtended = is;
    }
    
    public void setRollerVolts(double volts){
        rollerVolts = volts;
    }
    public void setFenceVolts(double volts){
        fenceVolts = volts;
    }
    
    public void setRollerBrakeOn(boolean is){
        roller.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }
    public void setFenceBrakeOn(boolean is){
        leftFence.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
        rightFence.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void log(){
        SmartDashboard.putNumber("Arm Degrees", getArmDegrees());
        SmartDashboard.putNumber("Arm Encoder", getEncoder());
        SmartDashboard.putBoolean("Arm Extended", getArmIsExtended());
        SmartDashboard.putBoolean("Arm Wants", armWantsExtended);
        SmartDashboard.putBoolean("Slider Extended", getSliderIsExtended());
        SmartDashboard.putBoolean("Slider Wants", sliderWantsExtended);
        SmartDashboard.putNumber("Fence Volts", fenceVolts);
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Intake", Status.PASSED);
    }
}
