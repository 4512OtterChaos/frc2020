/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoOptions;
import frc.robot.auto.Paths;
import frc.robot.commands.drive.TurnTo;
import frc.robot.commands.intake.SetIntakeLowered;
import frc.robot.commands.intake.SetSliderExtended;
import frc.robot.commands.shoot.SetShooterState;
import frc.robot.commands.superstructure.PrimeIntake;
import frc.robot.commands.superstructure.PrimeShooter;
import frc.robot.commands.superstructure.SimplerShootOuter;
import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.common.Constants;
import frc.robot.common.OCLEDManager;
import frc.robot.common.OCPhotonCam;
import frc.robot.common.OCXboxController;
import frc.robot.common.Testable;
import frc.robot.common.Constants.ShooterWristConstants;
import frc.robot.common.Constants.VisionConstants;
import frc.robot.common.OCXboxController.DriveMode;
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
    private OCPhotonCam photonShoot;
    private OCPhotonCam photonIntake;
    private OCLEDManager manager;
    
    private Paths paths;

    private SAS analysis;
    
    private Testable[] testableSystems;
    
    private OCXboxController driver = new OCXboxController(0);
    private OCXboxController operator;
    private boolean operatorConfigured = false;
        
    private AutoOptions autoOptions;
    private SendableChooser<DriveMode> driveModeChooser = new SendableChooser<DriveMode>();
    private boolean noDriverStation = true;
    
    public RobotContainer() {
        drivetrain = new Drivetrain();
        intake = new Intake();
        indexer = new Indexer();
        shooter = new Shooter();
        lift = new Lift();
        limelight = new Limelight(Configuration.PNP, 
            VisionConstants.kTranslation,
            VisionConstants.kShootHeight, 
            VisionConstants.kShootPitch, 
            VisionConstants.kTargetTranslation, 
            VisionConstants.kTargetHeight, 
            VisionConstants.kLatencyMsLime
        );
        photonShoot = new OCPhotonCam("photon-shoot", 
            VisionConstants.kShootHeight, 
            VisionConstants.kShootPitch, 
            VisionConstants.kTargetHeight
        );
        photonIntake = new OCPhotonCam("photon-intake", 
            VisionConstants.kIntakeHeight, 
            VisionConstants.kIntakePitch, 
            3.75
        );

        manager = new OCLEDManager(0, 120, OCLEDManager.Configuration.COPYSPLIT);
        
        shooter.setShooterBrakeOn(false);
        
        paths = new Paths(drivetrain.getLinearFF(), drivetrain.getKinematics());

        analysis = new SAS();
        
        testableSystems = new Testable[]{drivetrain};
        
        autoOptions = new AutoOptions(drivetrain, intake, indexer, shooter, limelight, photonIntake, analysis, paths);
        driveModeChooser.setDefaultOption("Curvature Drive", OCXboxController.DriveMode.CURVATURE);
        driveModeChooser.addOption("Curvature Volts", OCXboxController.DriveMode.CURVATUREVOLTS);
        driveModeChooser.addOption("Arcade Drive", OCXboxController.DriveMode.ARCADE);
        driveModeChooser.addOption("Tank Volts", OCXboxController.DriveMode.TANKVOLTS);
        

        configureButtonBindings();
    }
    /**
     * Runs every loop, regardless
     */
    public void periodic(){
        if(noDriverStation && DriverStation.getInstance().isDSAttached()){
            noDriverStation = false;
            autoOptions.submit();
            SmartDashboard.putData("Driving Mode", driveModeChooser);
        }

        manager.periodic();
    }
    private void configureButtonBindings(){
        configureDriverBindings();
    }
    private void configureDriverBindings(){
        configureDriveButtons(driver);

        /*
        new Trigger(()->driver.getTriggerAxis(Hand.kRight) > 0.3)
            .whenActive(
                SuperstructureCommands.intakeIndexBalls(intake, indexer, 6, 8)
            )
            .whenInactive(
                new InstantCommand(
                    ()->{
                        intake.setRollerVolts(0);
                        intake.setFenceVolts(0);
                        indexer.setVolts(0);
                    },
                    intake, indexer
                )
            );
        */

        new JoystickButton(driver, XboxController.Button.kB.value)
            .whenPressed(
                new SetIntakeLowered(intake, false)
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
        
        /*
        new Trigger(()->driver.getTriggerAxis(Hand.kLeft) > 0.3)
            .whenActive(
                SuperstructureCommands.shoot(drivetrain, intake, indexer, shooter, limelight, analysis)  
            )
            .whenInactive(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0);
                shooter.setShooterVelocity(0);
                shooter.setState(ShooterState.kIdleState);
            },
            drivetrain, shooter, indexer
            );
        */

        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(
                new SetShooterState(shooter, new ShooterState(30, 600))
                    .alongWith(SuperstructureCommands.feedShooter(indexer, intake, ()->true, 3))
            )
            .whenReleased(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0);
                shooter.setState(ShooterState.kIdleState);
            },
            drivetrain, shooter, indexer
            );

        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(
                SuperstructureCommands.safeIntake(intake)
            );
            
        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(
                TurnTo.createSimplerTurnToTarget(drivetrain, limelight)
                    .alongWith(new SetShooterState(shooter, analysis, limelight).withTimeout(0.75))
                    .andThen(
                        SuperstructureCommands.feedShooter(indexer, intake, ()->true, 3)
                        .alongWith(
                            //new PerpetualCommand(TurnTo.createSimpleTurnToTarget(drivetrain, limelight)),
                            new PerpetualCommand(TurnTo.createTensionedTurnToTarget(drivetrain, limelight)),
                            new PerpetualCommand(new SetShooterState(shooter, analysis, limelight))
                        )
                    )
            )
            .whenReleased(()->{
                drivetrain.tankDrive(0, 0);
                indexer.setVolts(0);
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
                    intake.setSliderIsExtended(true);
                    indexer.setVolts(-5);
                    intake.setFenceVolts(-12);
                }, intake, indexer
            )
            .whenReleased(
                ()->{
                    indexer.setVolts(0);
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
                new SetIntakeLowered(intake, true)  
            );
        new JoystickButton(operator, XboxController.Button.kB.value)
            .whenPressed(
                new SetIntakeLowered(intake, false)  
            );
        new JoystickButton(operator, XboxController.Button.kX.value)
            .whenPressed(
                new SetIntakeLowered(intake, false)
                .andThen(new SetSliderExtended(intake, false))
            );
    }
    private void configureDriveButtons(OCXboxController controller){
        RunCommand teleopDrive = new RunCommand(()->{

            DriveMode mode = getDriveMode();
            
            switch(mode){
                default:
                    drivetrain.setVelocityPercentage(controller.getLeftArcade(), controller.getRightArcade());
                break;
                case CURVATURE:
                    drivetrain.setVelocityPercentage(controller.getLeftCurvatureDrive(), controller.getRightCurvatureDrive());
                break;
                case CURVATUREVOLTS:
                    drivetrain.tankDrive(controller.getLeftCurvatureDrive(), controller.getRightCurvatureDrive());
                break;
                case TANKVOLTS:
                    drivetrain.tankDrive(controller.getY(Hand.kLeft), controller.getY(Hand.kRight));
                break;
                case HENRYDRIVEBRAKE:
                    drivetrain.tankDrive(controller.getY(Hand.kLeft, 0.8)*0.75*(1-controller.getTriggerAxis(Hand.kLeft)*0.75), controller.getY(Hand.kLeft, 0.8)*0.75*(1-controller.getTriggerAxis(Hand.kRight)*0.75));
                break;
                case HENRYDRIVEGAS:
                    drivetrain.tankDrive(controller.getTriggerAxis(Hand.kLeft)*0.75, controller.getTriggerAxis(Hand.kRight)*0.75);
                break;
            }
        }, drivetrain);
        drivetrain.setDefaultCommand(teleopDrive.beforeStarting(controller::resetLimiters));

        new JoystickButton(controller, XboxController.Button.kBumperRight.value)
            .whenPressed(()->controller.setDriveSpeed(OCXboxController.kSpeedFast))
            .whenReleased(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault));

        new JoystickButton(controller, XboxController.Button.kBumperLeft.value)
            .whenPressed(()->controller.setDriveSpeed(OCXboxController.kSpeedMax))
            .whenReleased(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault));
    }

    // ---------------------------------------------------------------------------------
    
    public Command getAutonomousCommand() {
        return autoOptions.getSelected();
    }
    public DriveMode getDriveMode(){
        return driveModeChooser.getSelected();
    }

    public void init(boolean auto){
        if(DriverStation.getInstance().getJoystickIsXbox(1) && !operatorConfigured){
            operator = new OCXboxController(1);
            configureOperatorBindings();
            operatorConfigured = true;
        }
        intake.init();
        lift.setRatchetEngaged(false);
        shooter.reset();
        shooter.setState(ShooterState.kIdleState);

        limelight.setConfiguration(Configuration.BASIC);

        if(auto){
            intake.setSliderIsExtended(false);
            indexer.setBrakeOn(false);
            shooter.setWristBrakeOn(false);
            shooter.setShooterBrakeOn(false);
            //drivetrain.resetOdometry(new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(9.125), new Rotation2d()), new Rotation2d());
        }
    }
    public void disable(){
        limelight.setConfiguration(Configuration.DRIVE);
    }
    
    public void setAllBrake(boolean is){
        drivetrain.setBrakeOn(is);
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
        drivetrain.log();

        NetworkTable liveTable = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
        
        Pose2d pose = drivetrain.getOdometry().getPoseMeters();
        liveTable.getEntry("robotX").setDouble(Units.metersToFeet(pose.getX()));
        liveTable.getEntry("robotY").setDouble(Units.metersToFeet(pose.getY()));
        liveTable.getEntry("robotHeading").setDouble(pose.getRotation().getRadians());

        intake.log();
        indexer.log();
        shooter.log();
        lift.log();
        limelight.log();
        SmartDashboard.putString("GS Type", photonIntake.findGSType().toString());
        /*
        if(photonIntake.hasTargets()){
            SmartDashboard.putNumber("PhotonIn Pitch", photonIntake.getLatestResult().getBestTarget().getPitch());
            SmartDashboard.putNumber("PhotonIn Dist", photonIntake.getBestDistance());
        }*/
    }
    
    public Testable[] getTestableSystems(){return testableSystems;};
}
