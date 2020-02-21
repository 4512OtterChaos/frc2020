/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.OCConfig;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import io.github.oblarg.oblog.Loggable;

public class Indexer extends SubsystemBase implements Loggable, Testable{

  private CANSparkMax bot;
  private CANSparkMax top;

  private DigitalInput frontBeam;
  private TimeOfFlight shooterFlight;
  private final double flightDefaultDistanceMM = 100;
  private final double flightDefaultErrorMM = 10;
  
  public Indexer() {
    bot = OCConfig.createMAX(13, ConfigType.INDEXER);
    top = OCConfig.createMAX(14, ConfigType.INDEXER);

    frontBeam = new DigitalInput(0);
    
    shooterFlight = new TimeOfFlight(0);
    shooterFlight.setRangingMode(RangingMode.Short, 24);
  }

  @Override
  public void periodic() {
  }

  public boolean getFrontBeam(){
    return frontBeam.get();
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
    top.setVoltage(volts);
  }
  public void setBotVolts(double volts){
    bot.setVoltage(volts);
  }
  public void setVolts(double topVolts, double botVolts){
    setTopVolts(topVolts);
    setBotVolts(botVolts);
  }

  public void setBrakeOn(boolean is){
    IdleMode mode = is ? IdleMode.kBrake : IdleMode.kCoast;
    OCConfig.setIdleMode(mode, bot);
    OCConfig.setIdleMode(mode, top);
}

  @Override
  public TestableResult test(){
    boolean flightGood = getFlightRangeMMError()<flightDefaultErrorMM;
    boolean sensorsGood = flightGood && !getFrontBeam();
    return new TestableResult("Indexer", Status.PASSED);
  }
}
