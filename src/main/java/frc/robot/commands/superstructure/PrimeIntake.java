/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PrimeIntake extends ParallelCommandGroup {

  public PrimeIntake(Intake intake, Indexer indexer) {
    super();
  }
  public PrimeIntake(Intake intake, Indexer indexer, Shooter shooter) {
    super();
  }
}
