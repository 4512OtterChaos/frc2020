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
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoOptions;
import frc.robot.auto.Paths;
import frc.robot.commands.drive.TurnTo;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.intake.IntakeUp;
import frc.robot.commands.intake.IntakeUpClear;
import frc.robot.commands.shoot.SetShooterState;
import frc.robot.commands.superstructure.IntakeIndexIncoming;
import frc.robot.commands.superstructure.PrimeIntake;
import frc.robot.commands.superstructure.PrimeShooter;
import frc.robot.commands.superstructure.SimplerShootOuter;
import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.common.Constants;
import frc.robot.common.OCLEDManager;
import frc.robot.common.OCXboxController;
import frc.robot.common.Testable;
import frc.robot.common.Constants.IntakeArmConstants;
import frc.robot.common.Constants.ShooterWristConstants;
import frc.robot.common.Constants.VisionConstants;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Limelight.Configuration;
import frc.robot.util.SAS;

public class RobotContainer {
    
    private Drivetrain drivetrain;
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private Lift lift;
    private Limelight limelight;
    private OCLEDManager manager;
    
    private Paths paths;

    private SAS sas;
    
    private Testable[] testableSystems;
    
    private OCXboxController driver = new OCXboxController(0);
    private OCXboxController operator;
    private boolean operatorConfigured = false;
        
    private AutoOptions autoOptions;

    private double testRPM = 0;
    private double testWristAngle = ShooterWristConstants.kClearIntakeDegrees;
    
    public RobotContainer() {
        drivetrain = new Drivetrain();
        intake = new Intake();
        indexer = new Indexer();
        shooter = new Shooter();
        lift = new Lift();
        limelight = new Limelight();

        manager = new OCLEDManager(0, 120, OCLEDManager.Configuration.SPLIT);
        
        shooter.setShooterBrakeOn(false);
        
        paths = new Paths(drivetrain.getLinearFF(), drivetrain.getKinematics());

        sas = new SAS();
        
        testableSystems = new Testable[]{drivetrain, limelight};
        
        autoOptions = new AutoOptions(drivetrain, intake, indexer, shooter, limelight);
        autoOptions.submit();
        
        configureButtonBindings();
    }
    
    private void configureButtonBindings(){
        configureDriverBindings();
    }
    private void configureDriverBindings(){
        RunCommand velocityControl = new RunCommand(()->drivetrain.setChassisSpeed(driver.getForward(), driver.getTurn(), true), drivetrain);
        drivetrain.setDefaultCommand(velocityControl);

        new JoystickButton(driver, XboxController.Button.kBumperRight.value)
            .whenPressed(()->drivetrain.setDriveSpeed(0.5))
            .whenReleased(()->drivetrain.setDriveSpeed(0.3));

        new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
            .whenPressed(()->drivetrain.setDriveSpeed(0.8))
            .whenReleased(()->drivetrain.setDriveSpeed(0.3));
        
        ConditionalCommand conditionalIntake = new ConditionalCommand(
            new PrimeIntake(intake, indexer, shooter),
            new InstantCommand(
                ()->intake.setSliderExtended(true),
                intake
            ).andThen(
                new IntakeIndexIncoming(intake, indexer, shooter)
            ), 
            ()->intake.getArmDegrees() > IntakeArmConstants.kLowerSafeDegrees
        );

        new Trigger(()->driver.getTriggerAxis(Hand.kRight) > 0.3)
            .whenActive(
                conditionalIntake
            )
            .whenInactive(
                new ConditionalCommand(
                    new InstantCommand(
                        ()->{
                            intake.setRollerVolts(0);
                            intake.setFenceVolts(0);
                            indexer.setVolts(0, 0);
                        },
                        intake, indexer
                    ),
                    new InstantCommand(),
                    ()->intake.getArmDegrees() < IntakeArmConstants.kLowerSafeDegrees
                )
            );

        new JoystickButton(driver, XboxController.Button.kB.value)
            .whenPressed(
                new IntakeUpClear(intake)
            );
        
        /*
        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(
                SuperstructureCommands.shoot(drivetrain, intake, indexer, shooter, limelight)
            )
            .whenReleased(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0, 0);
                shooter.setShooterVelocity(0);
            },
            drivetrain, shooter, indexer
            );*/
        
        new Trigger(()->driver.getTriggerAxis(Hand.kLeft) > 0.3)
            .whenActive(
                SuperstructureCommands.shoot(drivetrain, intake, indexer, shooter, limelight, sas)  
            )
            .whenInactive(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0, 0);
                shooter.setShooterVelocity(0);
                shooter.setState(ShooterState.kIdleState);
            },
            drivetrain, shooter, indexer
            );

        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(
                new SimplerShootOuter(drivetrain, intake, indexer, shooter, limelight, ShooterState.kClose)
            )
            .whenReleased(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0, 0);
                shooter.setState(ShooterState.kIdleState);
            },
            drivetrain, shooter, indexer
            );

        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(
                new SimplerShootOuter(drivetrain, intake, indexer, shooter, limelight, ShooterState.kTrenchLine)
            )
            .whenReleased(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0, 0);
                shooter.setState(ShooterState.kIdleState);
            },
            drivetrain, shooter, indexer
            );            

        new POVButton(driver, 0)
            .whenPressed(
                ()->lift.setVolts(12),
                lift
            )
            .whenReleased(
                ()->lift.setVolts(0),
                lift
            );
        new POVButton(driver, 180)
            .whenPressed(
                ()->lift.setVolts(-12),
                lift
            )
            .whenReleased(
                ()->lift.setVolts(0),
                lift
            );

        new JoystickButton(driver, XboxController.Button.kStickRight.value)
            .whenPressed(
                ()->{
                    intake.setRollerVolts(-6);
                }, intake
            )
            .whenReleased(
                ()->{
                    intake.setRollerVolts(0);
                }, intake
            );

        new JoystickButton(driver, XboxController.Button.kStickLeft.value)
            .whenPressed(
                ()->{
                    intake.setSliderExtended(true);
                    indexer.setVolts(-6, -6);
                    intake.setFenceVolts(-12);
                }, intake, indexer
            )
            .whenReleased(
                ()->{
                    indexer.setVolts(0,0);
                    intake.setFenceVolts(0);
                }, intake, indexer
            );

        new JoystickButton(driver, XboxController.Button.kStart.value)
            .whenPressed(
                new TurnTo(drivetrain, 0)
            );
        new JoystickButton(driver, XboxController.Button.kBack.value)
            .whenPressed(
                new TurnTo(drivetrain, 180)
            );
    }
    
    private void configureOperatorBindings(){ 
        new JoystickButton(operator, XboxController.Button.kA.value)
            .whenPressed(
                ()->lift.setRatchetEngaged(true)
            );
        new JoystickButton(operator, XboxController.Button.kB.value)
            .whenPressed(
                ()->lift.setRatchetEngaged(false)
            );

        new JoystickButton(operator, XboxController.Button.kX.value)
            .whenPressed(
                ()->intake.setRollerVolts(-8),
                intake
            )
            .whenReleased(
                ()->intake.setRollerVolts(0),
                intake
            );
    }
    
    public Command getAutonomousCommand() {
        return autoOptions.getSelected();
    }

    public void init(){
        if(DriverStation.getInstance().getJoystickIsXbox(1) && !operatorConfigured){
            operator = new OCXboxController(1);
            configureOperatorBindings();
            operatorConfigured = true;
        }
        intake.init();
        lift.setRatchetEngaged(false);
        shooter.reset();
        shooter.setState(ShooterState.kIdleState);

        limelight.setConfiguration(Configuration.PNP);
    }
    public void disable(){
        limelight.setConfiguration(Configuration.DRIVE);
    }
    
    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
        intake.setArmBrakeOn(is);
        intake.setFenceBrakeOn(is);
        intake.setRollerBrakeOn(is);
        indexer.setBrakeOn(is);
        shooter.setWristBrakeOn(is);
        if(DriverStation.getInstance().isFMSAttached()) lift.setBrakeOn(true);
        else lift.setBrakeOn(is);
    }
    public void stop(){
        drivetrain.tankDrive(0, 0);
    }
    
    public void log(){
        manager.periodic();
        
        drivetrain.log();
        intake.log();
        indexer.log();
        shooter.log();
        lift.log();
        limelight.log();
    }
    
    public Testable[] getTestableSystems(){return testableSystems;};
}
