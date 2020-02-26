/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.index.IndexHomeShooter;
import frc.robot.subsystems.Indexer;

public class PrimeShooter extends ParallelRaceGroup {
    
    public PrimeShooter(Indexer indexer) {
        super(
            new WaitCommand(2.5),
            new IndexHomeShooter(indexer)
        );
    }
}
