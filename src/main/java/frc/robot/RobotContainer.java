/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.Paths;
import frc.robot.common.Limelight;
import frc.robot.common.OCController;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
  
  private Drivetrain drivetrain;
  private Paths paths;

  private Limelight limelight;
  
  private OCController driver = new OCController(0);
  
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  
  private SendableChooser<Command> commandChooser  = new SendableChooser<>();
  
  public RobotContainer() {
    drivetrain = new Drivetrain();

    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
  }
  
  public Command getAutonomousCommand() {
    return commandChooser.getSelected();
  }
  
  public void setDriveCoast(boolean is){
    if(is) drivetrain.setIdleMode(IdleMode.kCoast);
    else drivetrain.setIdleMode(IdleMode.kBrake);
  }
  public void disable(){
    drivetrain.tankDrive(0, 0);
    setDriveCoast(true);
  }

  public void log(){
    led.setData(ledBuffer);

    Logger.updateEntries();
  }
}
