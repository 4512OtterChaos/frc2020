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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoOptions;
import frc.robot.auto.Paths;
import frc.robot.common.Limelight;
import frc.robot.common.OCController;
import frc.robot.common.Testable;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
  
  private Drivetrain drivetrain;
  private Paths paths;

  private Limelight limelight;

  private Testable[] testableSystems;
  
  private OCController driver = new OCController(0);
  private OCController operator;
  
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  
  private AutoOptions autoOptions;

  public RobotContainer() {
    drivetrain = new Drivetrain();

    autoOptions = new AutoOptions(drivetrain);

    if(DriverStation.getInstance().getJoystickIsXbox(1)) operator = new OCController(1);
    configureButtonBindings();

    autoOptions.submit();
    testableSystems = new Testable[]{drivetrain, limelight};
  }
  
  private void configureButtonBindings() {
  }
  
  public Command getAutonomousCommand() {
    return autoOptions.getSelected();
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

  public Testable[] getTestableSystems(){return testableSystems;};
}
