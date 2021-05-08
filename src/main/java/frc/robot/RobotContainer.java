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
        limelight = new Limelight(Configuration.PNP, VisionConstants.kTranslation, VisionConstants.kShootHeight,
                VisionConstants.kShootPitch, VisionConstants.kTargetTranslation, VisionConstants.kTargetHeight,
                VisionConstants.kLatencyMsLime);
        photonShoot = new OCPhotonCam("photon-shoot", VisionConstants.kShootHeight, VisionConstants.kShootPitch,
                VisionConstants.kTargetHeight);
        photonIntake = new OCPhotonCam("photon-intake", VisionConstants.kIntakeHeight, VisionConstants.kIntakePitch,
                3.75);

        manager = new OCLEDManager(0, 120, OCLEDManager.Configuration.COPYSPLIT);

        shooter.setShooterBrakeOn(false);

        paths = new Paths(drivetrain.getLinearFF(), drivetrain.getKinematics());

        analysis = new SAS();

        testableSystems = new Testable[] { drivetrain };

        autoOptions = new AutoOptions(drivetrain, intake, indexer, shooter, limelight, photonIntake, analysis, paths);
        driveModeChooser.setDefaultOption("Curvature Drive", OCXboxController.DriveMode.CURVATURE);
        driveModeChooser.addOption("Curvature Volts", OCXboxController.DriveMode.CURVATUREVOLTS);
        driveModeChooser.addOption("Arcade Drive", OCXboxController.DriveMode.ARCADE);
        driveModeChooser.addOption("Arcade Volts", OCXboxController.DriveMode.ARCADEVOLTS);
        driveModeChooser.addOption("Tank Volts", OCXboxController.DriveMode.TANKVOLTS);

        configureButtonBindings();
    }

    /**
     * Runs every loop, regardless
     */
    public void periodic() {
        if (noDriverStation && DriverStation.getInstance().isDSAttached()) {
            noDriverStation = false;
            autoOptions.submit();
            SmartDashboard.putData("Driving Mode", driveModeChooser);
        }

        manager.periodic();
    }

    private void configureButtonBindings() {
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        configureDriveButtons(driver);

        driver.rightTriggerButton.whenPressed(
            SuperstructureCommands.intakeIndexBalls(intake, indexer, 7, 8)
        )
        .whenReleased(new InstantCommand(() -> {
            intake.setRollerVolts(0);
            intake.setFenceVolts(0);
            indexer.setVolts(0);
        }, intake, indexer));

        driver.bButton.whenPressed(new SetIntakeLowered(intake, false));

        driver.leftTriggerButton.whenPressed(
            SuperstructureCommands.shoot(drivetrain, intake, indexer, shooter, limelight, analysis)
        )
        .whenReleased(() -> {
            drivetrain.tankDrive(0, 0);
            indexer.setVolts(0);
            shooter.setShooterVelocity(0);
            shooter.setState(ShooterState.kIdleState);
        }, drivetrain, shooter, indexer);

        driver.aButton.whenPressed(
            new SetShooterState(shooter, new ShooterState(30, 600))
            .alongWith(SuperstructureCommands.feedShooter(indexer, intake, () -> true, 3))
        )
        .whenReleased(() -> {
            drivetrain.tankDrive(0, 0);
            indexer.setVolts(0);
            shooter.setState(ShooterState.kIdleState);
        }, drivetrain, shooter, indexer);

        driver.xButton.whenPressed(SuperstructureCommands.safeIntake(intake));

        driver.yButton.whenPressed(
            TurnTo.createSimplerTurnToTarget(drivetrain, limelight)
            .alongWith(new SetShooterState(shooter, analysis, limelight).withTimeout(1))
            .andThen(
                SuperstructureCommands.feedShooter(indexer, intake, () -> true, 2.75)
                .alongWith(
                    // new PerpetualCommand(TurnTo.createSimpleTurnToTarget(drivetrain, limelight)),
                    // new PerpetualCommand(TurnTo.createTensionedTurnToTarget(drivetrain,
                    // limelight)),
                    new PerpetualCommand(new SetShooterState(shooter))
                )
            )
        )
        .whenReleased(() -> {
            drivetrain.tankDrive(0, 0);
            indexer.setVolts(0);
            shooter.setState(ShooterState.kIdleState);
        }, drivetrain, shooter, indexer);

        /*
        driver.povUpButton.whenPressed(() -> lift.setVolts(12), lift).whenReleased(() -> lift.setVolts(0), lift);
        driver.povDownButton.whenPressed(() -> lift.setVolts(-12), lift).whenReleased(() -> lift.setVolts(0), lift);
        */

        driver.stickRightButton.whenPressed(() -> {
            intake.setRollerVolts(-6);
        }, intake).whenReleased(() -> {
            intake.setRollerVolts(0);
        }, intake);

        driver.stickLeftButton.whenPressed(() -> {
            intake.setSliderIsExtended(true);
            indexer.setVolts(-5);
            intake.setFenceVolts(-12);
        }, intake, indexer).whenReleased(() -> {
            indexer.setVolts(0);
            intake.setFenceVolts(0);
        }, intake, indexer);

        driver.startButton.whenPressed(new TurnTo(drivetrain, 0));
        driver.backButton.whenPressed(new TurnTo(drivetrain, 180));
        driver.povDownButton.whenPressed(new SetShooterState(shooter, ShooterState.kLimp));

    }

    private void configureOperatorBindings() {
        operator.aButton.whenPressed(new SetIntakeLowered(intake, true));
        operator.bButton.whenPressed(new SetIntakeLowered(intake, false));
        operator.xButton.whenPressed(
            new SetIntakeLowered(intake, false)
            .andThen(new SetSliderExtended(intake, false))
        );
    }

    private void configureDriveButtons(OCXboxController controller) {
        RunCommand teleopDrive = new RunCommand(() -> {

            DriveMode mode = getDriveMode();

            switch (mode) {
                default: // ARCADE
                    drivetrain.setVelocityPercentage(controller.getLeftArcade(), controller.getRightArcade());
                    break;
                case ARCADEVOLTS:
                    drivetrain.tankDrive(controller.getLeftArcade(), controller.getRightArcade());
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
            }
        }, drivetrain);
        drivetrain.setDefaultCommand(teleopDrive.beforeStarting(controller::resetLimiters));

        controller.bumperLeftButton
        .whenPressed(() -> controller.setDriveSpeed(OCXboxController.kSpeedFast))
        .whenReleased(() -> controller.setDriveSpeed(OCXboxController.kSpeedDefault));

        controller.bumperRightButton
        .whenPressed(() -> controller.setDriveSpeed(OCXboxController.kSpeedMax))
        .whenReleased(() -> controller.setDriveSpeed(OCXboxController.kSpeedDefault));
    }

    // ---------------------------------------------------------------------------------

    public Command getAutonomousCommand() {
        return autoOptions.getSelected();
    }

    public DriveMode getDriveMode() {
        return driveModeChooser.getSelected();
    }

    public void init(boolean auto) {
        if (DriverStation.getInstance().getJoystickIsXbox(1) && !operatorConfigured) {
            operator = new OCXboxController(1);
            configureOperatorBindings();
            operatorConfigured = true;
        }
        intake.init();
        lift.setRatchetEngaged(false);
        shooter.reset();
        shooter.setState(ShooterState.kIdleState);

        limelight.nuke();
        limelight.setConfiguration(Configuration.PNP);

        if (auto) {
            /*
             * intake.setSliderIsExtended(false); indexer.setBrakeOn(false);
             * shooter.setWristBrakeOn(false); shooter.setShooterBrakeOn(false);
             */
            // drivetrain.resetOdometry(new Pose2d(Units.feetToMeters(2.5),
            // Units.feetToMeters(9.125), new Rotation2d()), new Rotation2d());
        }
    }

    public void disable() {
        limelight.setConfiguration(Configuration.DRIVE);
    }

    public void setAllBrake(boolean is) {
        drivetrain.setBrakeOn(is);
        intake.setFenceBrakeOn(is);
        intake.setRollerBrakeOn(is);
        indexer.setBrakeOn(is);
        shooter.setWristBrakeOn(is);
        if (DriverStation.getInstance().isFMSAttached())
            lift.setBrakeOn(true);
        else
            lift.setBrakeOn(is);
    }

    public void stop() {
        drivetrain.tankDrive(0, 0);
    }

    public void log() {
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
         * if(photonIntake.hasTargets()){ SmartDashboard.putNumber("PhotonIn Pitch",
         * photonIntake.getLatestResult().getBestTarget().getPitch());
         * SmartDashboard.putNumber("PhotonIn Dist", photonIntake.getBestDistance()); }
         */
    }

    public Testable[] getTestableSystems() {
        return testableSystems;
    };
}
