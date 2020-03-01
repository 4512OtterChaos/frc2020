/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.drive.TurnTo;
import frc.robot.commands.shoot.SetShooterState;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight.Configuration;

public class SimplerShootOuter extends SequentialCommandGroup {
    
    public SimplerShootOuter(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Limelight limelight, ShooterState shooterState) {
        super(
            new InstantCommand(
                ()->limelight.setConfiguration(Configuration.PNP)
                ),
            TurnTo.createSimplerTurnToTarget(drivetrain, limelight)
            .alongWith(
                new PrimeShooter(indexer, intake)
            )
            .alongWith(
                new StartEndCommand(
                    ()->shooter.setShooterVelocity(-1500),
                    ()->shooter.setShooterVelocity(0),
                    shooter
                )
                .withInterrupt(()->!indexer.getFlightBeam())
                .withTimeout(0.7)
            ),
            new SetShooterState(shooter, shooterState).withTimeout(1.25),
            new RunCommand(()->{
                if(shooter.checkIfStable()||!indexer.getFlightBeam()){
                    indexer.setVolts(4, 4);
                }
                else{
                    indexer.setVolts(0, 0);
                }
            }, indexer)
        );
    }
}
