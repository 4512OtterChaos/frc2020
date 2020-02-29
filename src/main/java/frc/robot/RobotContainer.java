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
import frc.robot.common.Constants;
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
    
    private Paths paths;

    private SAS sas;
    
    private Testable[] testableSystems;
    
    private OCXboxController driver = new OCXboxController(0);
    private OCXboxController operator;
    
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    
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
        
        shooter.setShooterBrakeOn(false);
        
        paths = new Paths(drivetrain.getFeedForward(), drivetrain.getKinematics());

        sas = new SAS();
        
        testableSystems = new Testable[]{drivetrain, limelight};
        
        autoOptions = new AutoOptions(drivetrain, intake, indexer, shooter, limelight);
        autoOptions.submit();
        
        if(DriverStation.getInstance().getJoystickIsXbox(1)) operator = new OCXboxController(1);
        configureButtonBindings();
    }
    
    private void configureButtonBindings(){
        //configureDriverBindings();
        configureCompBindings();
        if(operator != null) configureOperatorBindings();
    }
    private void configureCompBindings(){
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

        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(
                conditionalIntake
            )
            .whenReleased(
                ()->{
                    intake.setRollerVolts(0);
                    intake.setFenceVolts(0);
                    indexer.setVolts(0, 0);
                },
                intake, indexer
            );

        new JoystickButton(driver, XboxController.Button.kB.value)
            .whenPressed(
                new IntakeUpClear(intake)
            );

        new Trigger(()->driver.getTriggerAxis(Hand.kRight) > 0.3)
            .whenActive(
                TurnTo.createSimplerTurnToTarget(drivetrain, limelight)
                .alongWith(
                    new PrimeShooter(indexer, intake)
                    .alongWith(
                        new StartEndCommand(
                            ()->shooter.setShooterVelocity(-1500),
                            ()->shooter.setShooterVelocity(0),
                            shooter
                        )
                        .withInterrupt(()->!indexer.getFlightBeam())
                    )
                    .andThen(
                        new SetShooterState(shooter, new ShooterState(30, 3100))
                    )
                    .andThen(
                        new RunCommand(()->{
                            if(shooter.checkIfStable()){
                                indexer.setVolts(4, 4);
                            }
                            else{
                                indexer.setVolts(0, 0);
                            }
                        }, indexer)
                    )
                )
            )
            .whenInactive(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0, 0);
                shooter.setWristVolts(0);
                shooter.setShooterVelocity(0);
            },
            drivetrain, shooter, indexer
            );
    }
    private void configureDriverBindings() {
        RunCommand velocityControl = new RunCommand(()->drivetrain.setChassisSpeed(driver.getForward(), driver.getTurn(), true), drivetrain);
        /*
        drivetrain.setDefaultCommand(new RunCommand(()->{
            double left = driver.getLeftArcade();
            double right = driver.getRightArcade();
            SmartDashboard.putNumber("Left", left);
            SmartDashboard.putNumber("Right", right);
            drivetrain.tankDrive(left, right);
        }, drivetrain));
        */
        drivetrain.setDefaultCommand(velocityControl);
        
        new JoystickButton(driver, XboxController.Button.kBumperRight.value)
            .whenPressed(()->drivetrain.setDriveSpeed(0.5))
            .whenReleased(()->drivetrain.setDriveSpeed(0.3));

        new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
            .whenPressed(()->drivetrain.setDriveSpeed(0.8))
            .whenReleased(()->drivetrain.setDriveSpeed(0.3));

        /*
        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(()->{
                intake.setRollerVolts(8);
                intake.setFenceVolts(12);
            })
            .whenReleased(()->{
                intake.setRollerVolts(0);
                intake.setFenceVolts(0);
            });
        */

        /*
        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenHeld(new IndexIncoming(indexer)
            .alongWith(new InstantCommand(()->
                {
                    intake.setRollerVolts(9);
                    intake.setFenceVolts(12);
                }, intake)))
            .whenReleased(()->
                {
                    intake.setRollerVolts(0);
                    intake.setFenceVolts(0);
                }, intake);
        */
        new JoystickButton(driver, XboxController.Button.kA.value)
                .whenPressed(new IntakeIndexIncoming(intake, indexer, shooter))
                .whenReleased(()->{
                    intake.setRollerVolts(0);
                    intake.setFenceVolts(0);
                    indexer.setVolts(0, 0);
                }, intake, indexer);
        /*
        new JoystickButton(driver, XboxController.Button.kB.value)
            .whenPressed(()->indexer.setVolts(4, 4))
            .whenReleased(()->indexer.setVolts(0, 0));
        */
        new JoystickButton(driver, XboxController.Button.kB.value)
                .whenPressed(new PrimeShooter(indexer, shooter));
        
        /*
        new JoystickButton(driver, XboxController.Button.kB.value)
                .whenPressed(()->intake.setArmVolts(4), intake)
                .whenReleased(()->intake.setArmVolts(0), intake);
        */
        new JoystickButton(driver, XboxController.Button.kStickRight.value)
            .whenPressed(()->indexer.setVolts(-4, -4))
            .whenReleased(()->indexer.setVolts(0, 0));
        
        new JoystickButton(driver, XboxController.Button.kStickLeft.value)
                .toggleWhenPressed(new StartEndCommand(
                    ()->lift.setRatchetEngaged(true),
                    ()->lift.setRatchetEngaged(false)
                ));
        
        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(()->intake.setSliderExtended(!intake.getSliderExtended()));
        /*
        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(()->shooter.setShooterVelocity(4500))
            .whenReleased(()->shooter.setShooterVelocity(0));
        */
        new JoystickButton(driver, XboxController.Button.kY.value)
                .whenPressed(()->{
                    indexer.setVolts(3, 3);
                }, indexer)
                .whenReleased(()->{
                    indexer.setVolts(0, 0);
                }, indexer);

        new JoystickButton(driver, XboxController.Button.kBack.value)
            .whileActiveOnce(new PrimeIntake(intake,indexer,shooter));

        new JoystickButton(driver, XboxController.Button.kStart.value)
            .whenPressed(()->shooter.setShooterVelocity(3000))
            .whenReleased(()->shooter.setShooterVelocity(0));

        new Trigger(()->driver.getTriggerAxis(Hand.kLeft) > 0.2)
                .whenActive(()->lift.setVolts(-12), lift)
                .whenInactive(()->lift.setVolts(0), lift);

        new Trigger(()->driver.getTriggerAxis(Hand.kRight) > 0.2)
                .whenActive(()->lift.setVolts(12), lift)
                .whenInactive(()->lift.setVolts(0), lift);

        new POVButton(driver, 0)
            .whenPressed(()->shooter.setWristVolts(2.5))
            .whenReleased(()->shooter.setWristVolts(0));

        new POVButton(driver, 180)
            .whenPressed(()->shooter.setWristVolts(-2.5))
            .whenReleased(()->shooter.setWristVolts(0));
    }
    private void configureOperatorBindings(){
        
        intake.setDefaultCommand(new RunCommand(()->{
            double armVolts = operator.getForward()*12;
            intake.setArmVolts(armVolts);
        }, intake));

        
        new JoystickButton(operator, XboxController.Button.kX.value)
            .whenPressed(new SetShooterState(shooter, new ShooterState(31.5, 0)));
        
        new JoystickButton(operator, XboxController.Button.kY.value)
            .whenPressed(new SetShooterState(shooter, new ShooterState(41.5, 0)));

        new JoystickButton(operator, XboxController.Button.kA.value)
            .whenPressed(new IntakeDown(intake));
        
        new JoystickButton(operator, XboxController.Button.kB.value)
            .whenPressed(new IntakeUp(intake));

        new Trigger(()->operator.getTriggerAxis(Hand.kRight) > 0.2)
            .whenActive(TurnTo.createSimpleTurnToTarget(drivetrain, limelight))
                //.alongWith(new InstantCommand(()->limelight.setConfiguration(Configuration.PNP))))
            .whenInactive(()->{
                drivetrain.tankDrive(0,0);
                //limelight.setConfiguration(Configuration.DRIVE);
            }, drivetrain);
        
    }
    
    public Command getAutonomousCommand() {
        return autoOptions.getSelected();
    }

    public void init(){
        intake.init();

        limelight.setConfiguration(Configuration.PNP);
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
        //led.setData(ledBuffer);
        
        drivetrain.log();
        intake.log();
        indexer.log();
        shooter.log();
        lift.log();
        limelight.log();
    }
    
    public Testable[] getTestableSystems(){return testableSystems;};
}
