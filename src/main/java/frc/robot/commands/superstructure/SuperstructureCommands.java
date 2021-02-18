// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.drive.TurnTo;
import frc.robot.commands.index.IndexFeedShooter;
import frc.robot.commands.shoot.SetShooterState;
import frc.robot.states.ShooterState;
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
     * Prime robot for shooting by backdriving shooter and indexer past ToF sensor.
     */
    public static Command clearShooter(Shooter shooter, Indexer indexer){
        return new ConditionalCommand(
            new StartEndCommand(
                ()->{
                    shooter.setShooterVelocity(-1500);
                    indexer.setVolts(-3, -3);
                }, 
                ()->{
                    shooter.setShooterVelocity(0);
                    indexer.setVolts(0, 0);
                },
                shooter, indexer
            )
            .withInterrupt(()->!indexer.getFlightBeam())
            .withTimeout(0.8),
            new InstantCommand(),
            indexer::getFlightBeam
        );
    }

    public static Command shoot(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Limelight limelight, SAS analysis){
        return TurnTo.createSimplerTurnToTarget(drivetrain, limelight)
            .alongWith(
                clearShooter(shooter, indexer)
                .andThen(
                    new PrimeShooter(indexer, intake)
                    .alongWith(
                        new SetShooterState(shooter, analysis, limelight).withTimeout(1.25)
                    )
                )
            )
            .andThen(
                new IndexFeedShooter(indexer, ()->analysis.getIsReady(shooter, limelight, drivetrain, false))
                //new IndexFeedShooter(indexer, ()->)
                .alongWith(
                    new PerpetualCommand(TurnTo.createSimplerTurnToTarget(drivetrain, limelight))
                )
            )
            .alongWith(
                new RunCommand(
                    ()->analysis.getShotConfidence(shooter, limelight, drivetrain, false))  
            );
    }
    public static Command shoot(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Limelight limelight){
        return TurnTo.createSimplerTurnToTarget(drivetrain, limelight)
            .alongWith(
                clearShooter(shooter, indexer)
                .andThen(
                    new PrimeShooter(indexer, intake)
                    .alongWith(
                        new SetShooterState(shooter).withTimeout(1.25)
                    )
                )
            )
            .andThen(
                new IndexFeedShooter(indexer, ()->shooter.checkIfStable())
                .alongWith(
                    new PerpetualCommand(TurnTo.createSimpleTurnToTarget(drivetrain, limelight))
                )
            );
    }
}
