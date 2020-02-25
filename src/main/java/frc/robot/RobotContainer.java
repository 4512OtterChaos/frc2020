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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.auto.AutoOptions;
import frc.robot.auto.Paths;
import frc.robot.common.OCXboxController;
import frc.robot.common.Testable;
import frc.robot.subsystems.*;

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
    }
    
    private void configureButtonBindings() {
        RunCommand velocityControl = new RunCommand(()->drivetrain.setChassisSpeedPID(driver.getForward(), driver.getTurn()), drivetrain);
        
        drivetrain.setDefaultCommand(new RunCommand(()->{
            double left = driver.getLeftArcade();
            double right = driver.getRightArcade();
            SmartDashboard.putNumber("Left", left);
            SmartDashboard.putNumber("Right", right);
            drivetrain.tankDrive(left, right);
        }, drivetrain));
        
        new JoystickButton(driver, XboxController.Button.kBumperRight.value)
            .whenPressed(()->drivetrain.setDriveSpeed(0.5))
            .whenReleased(()->drivetrain.setDriveSpeed(0.3));

        new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
            .whenPressed(()->drivetrain.setDriveSpeed(0.8))
            .whenReleased(()->drivetrain.setDriveSpeed(0.3));

        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(()->{
                intake.setRollerVolts(8);
                intake.setFenceVolts(12);
            })
            .whenReleased(()->{
                intake.setRollerVolts(0);
                intake.setFenceVolts(0);
            });

        new JoystickButton(driver, XboxController.Button.kB.value)
            .whenPressed(()->indexer.setVolts(4, 4))
            .whenReleased(()->indexer.setVolts(0, 0));

        new JoystickButton(driver, XboxController.Button.kStickRight.value)
            .whenPressed(()->indexer.setVolts(-4, -4))
            .whenReleased(()->indexer.setVolts(0, 0));
        
        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(()->intake.setSliderExtended(!intake.getSliderExtended()));

        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(()->shooter.setShooterPID(4000))
            .whenReleased(()->shooter.setShooterVolts(0));

        new POVButton(driver, 0)
            .whenPressed(()->shooter.setWristVolts(2.5))
            .whenReleased(()->shooter.setWristVolts(0));

        new POVButton(driver, 180)
            .whenPressed(()->shooter.setWristVolts(-2.5))
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
        //led.setData(ledBuffer);
        
        drivetrain.log();
        intake.log();
        indexer.log();
        shooter.log();
        lift.log();
    }
    
    public Testable[] getTestableSystems(){return testableSystems;};
}
