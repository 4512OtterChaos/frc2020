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
import frc.robot.common.OCXboxController;
import frc.robot.common.Testable;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
  
  private Drivetrain drivetrain;
  private Intake intake;
  private Indexer indexer;
  private Lift lift;
  private Limelight limelight;

  private Paths paths;

  private Testable[] testableSystems;
  
  private OCXboxController driver = new OCXboxController(0);
  private OCXboxController operator;
  
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  
  private AutoOptions autoOptions;

  public RobotContainer() {
    drivetrain = new Drivetrain();
    intake = new Intake();
    indexer = new Indexer();
    lift = new Lift();
    limelight = new Limelight();

    paths = new Paths(drivetrain.getFeedForward(), drivetrain.getKinematics());

    testableSystems = new Testable[]{drivetrain, limelight};

    autoOptions = new AutoOptions(drivetrain, intake, indexer);
    autoOptions.submit();

    if(DriverStation.getInstance().getJoystickIsXbox(1)) operator = new OCXboxController(1);
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
  }
  
  public Command getAutonomousCommand() {
    return autoOptions.getSelected();
  }
  
  public void disable(){
    drivetrain.tankDrive(0, 0);
    drivetrain.setBrakeOn(false);
    lift.setBrakeOn(false);
  }

  public void log(){
    led.setData(ledBuffer);

    Logger.updateEntries();
  }

  public Testable[] getTestableSystems(){return testableSystems;};
}
