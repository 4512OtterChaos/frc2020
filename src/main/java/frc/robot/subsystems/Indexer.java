/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.IndexerConstants.*;

import frc.robot.common.Constants;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;

public class Indexer extends SubsystemBase implements Testable{
    
    private CANSparkMax left = new CANSparkMax(8, MotorType.kBrushless);
    private CANSparkMax right = new CANSparkMax(9, MotorType.kBrushless);

    private CANEncoder leftEncoder = left.getEncoder();
    //private CANEncoder rightEncoder = right.getEncoder();

    //private PIDController topController = new PIDController(kP, kI, kD, Constants.kRobotDelta);
    //private PIDController botController = new PIDController(kP, kI, kD, Constants.kRobotDelta);
    
    // Proximity sensor for receiving powercells
    private final DigitalInput receiveBeam = new DigitalInput(2);
    private final DigitalInput shootBeam = new DigitalInput(3);
    //private final TimeOfFlight shooterFlight = new TimeOfFlight(0);
    //private final double flightDefaultDistanceMM = 140;
    //private final double flightDefaultErrorMM = 10;

    private double volts = 0;
    
    public Indexer() {        
        OCConfig.configMotors(ConfigType.INDEXER, left, right);

        left.setInverted(false);
        right.setInverted(false);

        //shooterFlight.setRangingMode(RangingMode.Short, 24);
    }
    
    @Override
    public void periodic() {
        left.setVoltage(volts);
        right.setVoltage(-volts);
    }
    
    public boolean getReceiveBeam(){
        return !receiveBeam.get();
    }
    public boolean getShootBeam(){
        return !shootBeam.get();
    }
    /*
    public double getFlightRangeMM(){
        return shooterFlight.getRange();
    }
    public double getFlightRangeMMError(){
        return shooterFlight.getRangeSigma();
    }
    public boolean getFlightBeam(){
        return getFlightRangeMM()<flightDefaultDistanceMM;
    }
    */
    
    public void setVolts(double volts){
        this.volts = volts;
    }
    
    public void setBrakeOn(boolean is){
        IdleMode mode = is ? IdleMode.kBrake : IdleMode.kCoast;
        OCConfig.setIdleMode(mode, left, right);
    }
    
    public void log(){
        SmartDashboard.putBoolean("Receive Beam", getReceiveBeam());
        SmartDashboard.putBoolean("Shoot Beam", getShootBeam());
        //SmartDashboard.putNumber("Flight Distance", getFlightRangeMM());
        SmartDashboard.putNumber("Indexer Volts", volts);
        SmartDashboard.putNumber("Indexer RPM", leftEncoder.getVelocity());
    }

    @Override
    public TestableResult test(){
        //boolean flightGood = getFlightRangeMMError()<flightDefaultErrorMM;
        //boolean sensorsGood = flightGood && !getFrontBeam();
        return new TestableResult("Indexer", Status.PASSED);
    }
}
