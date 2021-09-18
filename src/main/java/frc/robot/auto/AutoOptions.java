/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.photonvision.LEDMode;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.Paths.GSType;
import frc.robot.commands.auto.GalacticSearchRamsete;
import frc.robot.commands.auto.PhotonWaypointRamsete;
import frc.robot.commands.auto.StandardRamseteCommand;
import frc.robot.commands.drive.TurnTo;
import frc.robot.commands.intake.SetIntakeLowered;
import frc.robot.commands.intake.SetSliderExtended;
import frc.robot.commands.shoot.SetShooterState;
import frc.robot.commands.superstructure.SimplerShootOuter;
import frc.robot.commands.superstructure.SuperstructureCommands;
import frc.robot.common.OCPhotonCam;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.SAS;

/**
* Class for holding commandGroups defining different auto options.
*/
public class AutoOptions {
    public enum StartingOrientation{
        FORWARD,
        BACKWARD
    }
    // Chooser boxes for creating autos
    SendableChooser<StartingOrientation> startingOrientation = new SendableChooser<>();
    SendableChooser<Command> stage1Options = new SendableChooser<>();
    SendableChooser<Command> stage2Options = new SendableChooser<>();
    SendableChooser<Command> stage3Options = new SendableChooser<>();
    SendableChooser<Command> stage4Options = new SendableChooser<>();
    List<SendableChooser<Command>> stageOptions = new ArrayList<SendableChooser<Command>>(
        List.of(
            stage1Options,
            stage2Options,
            stage3Options,
            stage4Options
        )
    );
    SendableChooser<Command> fullAutoOptions = new SendableChooser<>();

    private GSType gsType = GSType.FAIL; // we store this to increase scope at auto init
    
    // Maps of modular commands as well as complete preset auto command groups
    //HashMap<String, Command> stageOptions = new HashMap<>();
    //HashMap<String, Command> fullOptions = new HashMap<>();
    
    /**
    * Constructs different auto options given subsystems.
    */
    public AutoOptions(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, OCPhotonCam photonShoot, SAS analysis, Paths paths){
        
        SimpleMotorFeedforward driveFF = drivetrain.getLinearFF();
        DifferentialDriveKinematics driveKin = drivetrain.getKinematics();
        // add modular/preset commands
        
        
        for(SendableChooser<Command> chooser:stageOptions){
            chooser.setDefaultOption("Nothing",
                new InstantCommand(()->drivetrain.tankDrive(0,0), drivetrain)
            );
            chooser.addOption("1m Backward", 
                new StandardRamseteCommand(drivetrain, new OCPath(
                    List.of(
                        new Pose2d(0, 0, new Rotation2d()),
                        new Pose2d(1, 0, new Rotation2d())
                    ), driveFF, driveKin).getReversed()
                )
            );
            
            chooser.addOption("Simple Short Backward(Timed)",
                new StartEndCommand(
                    ()->{
                        drivetrain.setChassisSpeed(-0.3, 0);
                    },
                    ()->drivetrain.tankDrive(0, 0),
                    drivetrain
                ).withTimeout(0.75)
            );
            
            /*
            chooser.addOption("Simpler Shoot", 
                new SimplerShootOuter(drivetrain, intake, indexer, shooter, photonShoot, new ShooterState(30, 3000))
            );
            */
        }

        //---------------- preset autos

        fullAutoOptions.setDefaultOption("Nothing",
            new InstantCommand(()->drivetrain.tankDrive(0,0), drivetrain)
        );

        /*
        fullAutoOptions.addOption("Example", 
            new StandardRamseteCommand(drivetrain, paths.example)
        );

        fullAutoOptions.addOption("Example, then inverted",
            new StandardRamseteCommand(drivetrain, paths.example)
            .andThen(
                new StandardRamseteCommand(drivetrain, paths.example.getInverted())
            )
        );
        */

        fullAutoOptions.addOption("Simple Back then Shoot(Timed)",
            new StartEndCommand(
                ()->{
                    drivetrain.setChassisSpeed(-0.3, 0);
                },
                ()->drivetrain.tankDrive(0, 0),
                drivetrain
            ).withTimeout(0.75)
            .andThen(
                SuperstructureCommands.shoot(drivetrain, intake, indexer, shooter, photonShoot, analysis)
                .alongWith(
                    new InstantCommand(()->photonShoot.setLED(LEDMode.kDefault))
                )
                .andThen(
                    new SetShooterState(shooter, ShooterState.kIdleState)
                )
            )
        );
        
        /*
        fullAutoOptions.addOption("Galactic Search but COOL B)",
            SuperstructureCommands.intakeIndexBalls(intake, indexer, 9, 8)
            .alongWith(
                new WaitCommand(0.2).andThen(
                    new SetShooterState(shooter, ShooterState.kLimp).withTimeout(0.25).alongWith(
                        new PhotonWaypointRamsete(photonIntake, drivetrain)
                    )
                )
            ).withTimeout(8)
            .andThen(
                new InstantCommand(()->drivetrain.setBrakeOn(false))
            )
        );
        

        fullAutoOptions.addOption("Slalom",
            new StandardRamseteCommand(drivetrain, paths.slalom)
            .beforeStarting(()->
                drivetrain.resetOdometry(
                    paths.slalom.getInitialPose()
                ), drivetrain
            )
        );

        fullAutoOptions.addOption("Barrel",
            new StandardRamseteCommand(drivetrain, paths.barrel)
            .beforeStarting(()->
                drivetrain.resetOdometry(
                    paths.barrel.getInitialPose()
                ), drivetrain
            )
        );

        fullAutoOptions.addOption("Bounce", 
            new StandardRamseteCommand(drivetrain, paths.bounce1)
            .alongWith(
                new SetSliderExtended(intake, true)
                .andThen(new SetIntakeLowered(intake, true))
            )
            .beforeStarting(()->
                drivetrain.resetOdometry(
                    paths.bounce1.getInitialPose()
                ), drivetrain
            )
            .andThen(
                new StandardRamseteCommand(drivetrain, paths.bounce2)
                .alongWith(new SetIntakeLowered(intake, false))
            )
            .andThen(
                new StandardRamseteCommand(drivetrain, paths.bounce3)
                .alongWith(new SetIntakeLowered(intake, true))
            )
            .andThen(new StandardRamseteCommand(drivetrain, paths.bounce4))
            .andThen(new SetIntakeLowered(intake, false),  new SetSliderExtended(intake, false))
        );

        fullAutoOptions.addOption("Galactic Search",
            new InstantCommand(()->{
                gsType = photonIntake.findGSType();
                drivetrain.resetOdometry(
                    paths.getGSPath(gsType).getInitialPose()
                );
            }, drivetrain)
            .andThen(
                new WaitCommand(0.25)
            )
            .andThen(
                new GalacticSearchRamsete(drivetrain, paths, photonIntake)
            )
            .alongWith(
                new SetSliderExtended(intake, true).andThen(
                    SuperstructureCommands.intakeIndexBalls(intake, indexer, 12, 8.5)
                )
            )
        );
        */
        
        // populate sendable choosers with constructed commands
        //putStageDefaultOption("Nothing", nothing);
        //fullAutoOptions.setDefaultOption("Nothing", nothing);
        
        /*
        for(Entry<String,Command> entry:stageOptions.entrySet()){
            putStageOption(entry.getKey(), entry.getValue());
        }
        for(Entry<String,Command> entry:fullOptions.entrySet()){
            fullAutoOptions.addOption(entry.getKey(), entry.getValue());
        }
        */
        
    }
    
    /*
    private void putStageOption(String name, Command command){
        stage1Options.addOption(name, command);
        stage2Options.addOption(name, command);
        stage3Options.addOption(name, command);
        stage4Options.addOption(name, command);
    }
    private void putStageDefaultOption(String name, Command command){
        stage1Options.setDefaultOption(name, command);
        stage2Options.setDefaultOption(name, command);
        stage3Options.setDefaultOption(name, command);
        stage4Options.setDefaultOption(name, command);
    }
    */
    
    /**
    * Constructs a command group based off of selected stage commands.
    * If a preset auto is selected, it will use that instead.
    */
    public Command getSelected(){
        Command selected;
        /*
        if(fullAutoOptions.getSelected().getName(){
            selected = 
            stage1Options.getSelected().andThen(
            stage2Options.getSelected()).andThen(
            stage3Options.getSelected()).andThen(
            stage4Options.getSelected())
            .withInterrupt(()->Paths.getHasAbandonedTrajectory());
        }
        else{
            selected = 
            fullAutoOptions.getSelected()
            .withInterrupt(()->Paths.getHasAbandonedTrajectory());
        }
        */
        selected = fullAutoOptions.getSelected();
        return selected;
    }
    /**
    * Display sendable choosers
    */
    public void submit(){
        SmartDashboard.putData("Starting Orientation", startingOrientation);
        SmartDashboard.putData("Stage 1 Options", stage1Options);
        SmartDashboard.putData("Stage 2 Options", stage2Options);
        SmartDashboard.putData("Stage 3 Options", stage3Options);
        SmartDashboard.putData("Stage 4 Options", stage4Options);
        SmartDashboard.putData("Full Auto Options", fullAutoOptions);
    }
}
