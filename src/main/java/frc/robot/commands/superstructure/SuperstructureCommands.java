// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.drive.TurnTo;
import frc.robot.commands.index.IndexFeedShooter;
import frc.robot.commands.index.IndexHomeShooter;
import frc.robot.commands.index.IndexIncoming;
import frc.robot.commands.intake.SetIntakeLowered;
import frc.robot.commands.intake.SetSliderExtended;
import frc.robot.commands.shoot.SetShooterState;
import frc.robot.common.OCPhotonCam;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.SAS;

/**
 * Factory class for creating superstructure commands using inline command groups.
 */
public class SuperstructureCommands {

    /**
     * Lowers intake, then starts intaking and indexing power cells.
     */
    public static Command intakeIndexBalls(Intake intake, Indexer indexer, double rollerVolts, double fenceVolts){
        return new SetIntakeLowered(intake, true)
            /*
            .alongWith(
                new IndexHomeIntake(indexer)
                    .withTimeout(0.3)
            )*/
            .alongWith(
                new StartEndCommand(
                    ()->{
                        intake.setRollerVolts(rollerVolts);
                        intake.setFenceVolts(fenceVolts);
                    }, 
                    ()->{
                        intake.setRollerVolts(0);
                        intake.setFenceVolts(0);
                    }
                )
                .alongWith(
                    new IndexIncoming(indexer)
                )
            );
    }

    public static Command safeIntake(Intake intake){
        return new SetIntakeLowered(intake, false)
            .andThen(
                new WaitUntilCommand(()->!intake.getArmIsLowered())
                .andThen(new SetSliderExtended(intake, false))
            );
    }

    /**
     * Prime robot for shooting by moving indexed powercells to the shooting sensor.
     */
    public static Command primeShooter(Indexer indexer, Intake intake){
        return new IndexHomeShooter(indexer)
            .withTimeout(0.75)
            .alongWith(
                new InstantCommand((()->intake.setSliderIsExtended(true)), intake)
            );
    }

    /**
     * Prepare robot for shooting by backdriving shooter and indexer past shooting sensor.
     */
    public static Command clearShooter(Shooter shooter, Indexer indexer){
        return new ConditionalCommand(
            new StartEndCommand(
                ()->{
                    shooter.setShooterVelocity(-1500);
                    indexer.setVolts(-4);
                }, 
                ()->{
                    shooter.setShooterVelocity(0);
                    indexer.setVolts(0);
                },
                shooter, indexer
            )
            .withInterrupt(()->!indexer.getShootBeam())
            .withTimeout(0.8),
            new InstantCommand(),
            indexer::getShootBeam
        );
    }

    public static Command feedShooter(Indexer indexer, Intake intake, BooleanSupplier isReady, double indexVolts){
        return new IndexFeedShooter(indexer, isReady, indexVolts)
        .deadlineWith(
            new FunctionalCommand( // set fence while feeding as well
                ()->{
                    intake.setSliderIsExtended(true);
                    intake.setFenceVolts(0);
                },
                ()->intake.setFenceVolts(8),
                (interrupted)->intake.setFenceVolts(0),
                ()->false, // this is interrupted by IndexFeedShooter
                intake
            )
        );
    }

    /**
     * Turns to target while priming the indexer to feed and setting optimal shooter state, then fires when ready
     */
    public static Command shoot(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, OCPhotonCam camera, SAS analysis){
        return TurnTo.createSimplerTurnToTarget(drivetrain, camera)
        //return TurnTo.createTensionedTurnToTarget(drivetrain, camera)
            .alongWith(
                clearShooter(shooter, indexer)
                .andThen(
                    primeShooter(indexer, intake)
                    .alongWith(
                        new SetShooterState(shooter, analysis, camera).withTimeout(1.75)
                    )
                )
            )
            .andThen(
                feedShooter(indexer, intake, ()->analysis.getIsReady(camera.getBestDistanceInches(), shooter, drivetrain), 3)
                .alongWith(
                    //new PerpetualCommand(TurnTo.createSimpleTurnToTarget(drivetrain, camera)),
                    //new PerpetualCommand(TurnTo.createTensionedTurnToTarget(drivetrain, camera)),
                    new PerpetualCommand(new SetShooterState(shooter))
                )
            );
    }
}
