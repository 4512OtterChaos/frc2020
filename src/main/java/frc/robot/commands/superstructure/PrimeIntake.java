/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.index.IndexHomeIntake;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class PrimeIntake extends ParallelCommandGroup {

  public PrimeIntake(Intake intake, Indexer indexer) {
    super(
        new IntakeDown(intake),
        new ParallelRaceGroup(
            new WaitCommand(2.5),
            new IndexHomeIntake(indexer)
        )
    );
  }
}
