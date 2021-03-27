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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.index.IndexHomeIntake;
import frc.robot.commands.intake.SetIntakeLowered;
import frc.robot.commands.shoot.BackdriveShooterBalls;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PrimeIntake extends SequentialCommandGroup {
    
    public PrimeIntake(Intake intake, Indexer indexer, Shooter shooter) {
        super(
            new ConditionalCommand(
                new SetIntakeLowered(intake, true),
                new InstantCommand(), 
                intake::getArmIsLowered
            ),
            new IndexHomeIntake(indexer)
                .alongWith(
                    new BackdriveShooterBalls(shooter, indexer::getShootBeam)
                )
                .withTimeout(2)
        );
    }
}
