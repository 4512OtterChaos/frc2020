/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.AutoOptions;
import frc.robot.auto.Paths;
import frc.robot.common.OCXboxController;
import frc.robot.common.Testable;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    
    private Drivetrain drivetrain;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
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
        shooter = new Shooter();
        lift = new Lift();
        limelight = new Limelight();
        
        shooter.setShooterBrakeOn(false);
        
        paths = new Paths(drivetrain.getFeedForward(), drivetrain.getKinematics());
        
        testableSystems = new Testable[]{drivetrain, limelight};
        
        autoOptions = new AutoOptions(drivetrain, intake, indexer, shooter, limelight);
        autoOptions.submit();
        
        if(DriverStation.getInstance().getJoystickIsXbox(1)) operator = new OCXboxController(1);
        configureButtonBindings();

        Logger.configureLoggingAndConfig(this, false);
    }
    
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new RunCommand(()->{
            double left = driver.getLeftArcade();
            double right = driver.getRightArcade();
            drivetrain.tankDrive(left, right);
        }, drivetrain));
        
        new JoystickButton(driver, XboxController.Button.kBumperRight.value)
            .whenPressed(()->drivetrain.setDriveSpeed(0.5))
            .whenReleased(()->drivetrain.setDriveSpeed(0.3));

        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(()->{
                intake.setRollerVolts(2);
                intake.setFenceVolts(2);
            })
            .whenReleased(()->{
                intake.setRollerVolts(0);
                intake.setFenceVolts(0);
            });

        new JoystickButton(driver, XboxController.Button.kB.value)
            .whenPressed(()->indexer.setVolts(2, 2))
            .whenReleased(()->indexer.setVolts(0, 0));
        
        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(()->intake.setSliderExtended(true))
            .whenReleased(()->intake.setSliderExtended(false));

        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(()->shooter.setWristVolts(2))
            .whenReleased(()->shooter.setWristVolts(0));
    }
    
    public Command getAutonomousCommand() {
        return autoOptions.getSelected();
    }
    
    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
        intake.setArmBrakeOn(is);
        intake.setFenceBrakeOn(is);
        intake.setRollerBrakeOn(is);
        indexer.setBrakeOn(is);
        shooter.setWristBrakeOn(is);
        lift.setBrakeOn(is);
    }
    public void stop(){
        drivetrain.tankDrive(0, 0);
    }
    
    public void log(){
        led.setData(ledBuffer);
        
        Logger.updateEntries();
    }
    
    public Testable[] getTestableSystems(){return testableSystems;};
}
