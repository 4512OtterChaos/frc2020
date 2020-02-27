/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.index.IndexHomeShooter;
import frc.robot.commands.shoot.BackdriveShooterBalls;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class PrimeShooter extends ParallelCommandGroup {
    
    public PrimeShooter(Indexer indexer, Shooter shooter) {
        super(
            new IndexHomeShooter(indexer)
                .alongWith(
                    new BackdriveShooterBalls(shooter, indexer::getFlightBeam)
                )
                .withTimeout(2.5)
        );
    }
}
