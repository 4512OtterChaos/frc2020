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
    
    private CANSparkMax bot = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax top = new CANSparkMax(14, MotorType.kBrushless);

    private CANEncoder topEncoder = new CANEncoder(top);
    private CANEncoder botEncoder = new CANEncoder(bot);

    private PIDController topController = new PIDController(kP, kI, kD, Constants.kRobotDelta);
    private PIDController botController = new PIDController(kP, kI, kD, Constants.kRobotDelta);
    
    private final DigitalInput frontBeam = new DigitalInput(2);
    private final TimeOfFlight shooterFlight = new TimeOfFlight(0);
    private final double flightDefaultDistanceMM = 140;
    private final double flightDefaultErrorMM = 10;
    
    public Indexer() {        
        OCConfig.configMotors(ConfigType.INDEXER, bot, top);

        shooterFlight.setRangingMode(RangingMode.Short, 24);
    }
    
    @Override
    public void periodic() {
    }
    
    public boolean getFrontBeam(){
        return !frontBeam.get();
    }
    public double getFlightRangeMM(){
        return shooterFlight.getRange();
    }
    public double getFlightRangeMMError(){
        return shooterFlight.getRangeSigma();
    }
    public boolean getFlightBeam(){
        return getFlightRangeMM()<flightDefaultDistanceMM;
    }
    
    public void setTopVolts(double volts){
        top.setVoltage(-volts);
    }
    public void setBotVolts(double volts){
        bot.setVoltage(volts);
    }
    public void setVolts(double topVolts, double botVolts){
        setTopVolts(topVolts);
        setBotVolts(botVolts);
    }

    public void setTopVelocity(double rpm){
        
    }
    
    public void setBrakeOn(boolean is){
        IdleMode mode = is ? IdleMode.kBrake : IdleMode.kCoast;
        OCConfig.setIdleMode(mode, bot, top);
    }
    
    public void log(){
        SmartDashboard.putBoolean("Front Beam", getFrontBeam());
        SmartDashboard.putBoolean("Flight Beam", getFlightBeam());
        SmartDashboard.putNumber("Flight Distance", getFlightRangeMM());
        SmartDashboard.putNumber("Top RPM", topEncoder.getVelocity());
        SmartDashboard.putNumber("Bot RPM", botEncoder.getVelocity());
    }

    @Override
    public TestableResult test(){
        boolean flightGood = getFlightRangeMMError()<flightDefaultErrorMM;
        boolean sensorsGood = flightGood && !getFrontBeam();
        return new TestableResult("Indexer", Status.PASSED);
    }
}
