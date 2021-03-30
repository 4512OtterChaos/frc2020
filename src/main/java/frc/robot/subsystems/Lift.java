/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.*;
import static frc.robot.common.Constants.LiftConstants.*;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;

public class Lift extends SubsystemBase implements Testable{
    
    //private CANSparkMax master = new CANSparkMax(8, MotorType.kBrushless);
    //private CANSparkMax slave = new CANSparkMax(9, MotorType.kBrushless);
    
    private double volts = 0;
    
    //private DoubleSolenoid ratchet = new DoubleSolenoid(2, 3);
        
    //private DigitalInput botSwitch;
    //private DigitalInput ratchetSwitch;
    
    private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelerationFF);
    
    private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kVelocityConstraint, kAccelerationConstraint), kRobotDelta); // Positional PID controller
    
    public Lift() {
        //OCConfig.configMotors(ConfigType.LIFT, master, slave);
        //botSwitch = new DigitalInput(0);
        //ratchetSwitch = new DigitalInput(0);
        
        //master.setInverted(false);
        //OCConfig.setStatusSlow(slave);
    }
    
    @Override
    public void periodic() {
        /*
        double minVolts = -12;
        double maxVolts = 12;
        if(encoder.getPosition()<=kMinHeightRotations) minVolts = 0;
        if(encoder.getPosition()>=kMaxHeightRotations) maxVolts = 0;
        if(getRatchetEngaged()){
            maxVolts = 0.5;
        }
        volts = MathHelp.clamp(volts, minVolts, maxVolts);
        master.setVoltage(volts);
        slave.setVoltage(volts);
        */
    }
    
    /*
    public boolean getBotSwitch(){
        return botSwitch.get();
    }
    public boolean getRatchetSwitch(){
        return ratchetSwitch.get();
    }
    */
    public boolean getRatchetEngaged(){
        //return ratchet.get()!=Value.kForward;
        return false;
    }
    
    public ProfiledPIDController getController(){
        return controller;
    }
    public double getRotations(){
        return 0;
    }
    
    public void setVolts(double volts){
        this.volts = volts;
    }
    
    /**
    * Sets the controller goal.
    * @param rotations Motor rotations setpoint
    */
    public void setPID(double rotations){
        //double volts = controller.calculate(encoder.getPosition(), rotations);
        //volts += feedForward.calculate(controller.getGoal().velocity);
        //setVolts(volts);
    }
    
    public void setRatchetEngaged(boolean engaged){
        //ratchet.set(engaged ? Value.kReverse : Value.kForward);
    }
    
    public void setBrakeOn(boolean is){
        //OCConfig.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast, master, slave);
    }
    
    public void log(){
        //SmartDashboard.putNumber("Lift Encoder", encoder.getPosition());
        //SmartDashboard.putNumber("Lift Master Percent", master.getAppliedOutput());
        //SmartDashboard.putNumber("Lift Slave Percent", slave.getAppliedOutput());
        SmartDashboard.putBoolean("Lift Ratchet Engaged", getRatchetEngaged());
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Lift", Status.PASSED);
    }
}
