/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.index;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class IndexHomeShooter extends CommandBase {
  
  final Indexer indexer;
  final double kVolts = 3;

  boolean findHome;
  boolean foundHome;
  /**
   * Creates a new IndexIncoming.
   */
  public IndexHomeShooter(Indexer indexer) {
    this.indexer = indexer;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    findHome = false;
    foundHome = false;
  }

  @Override
  public void execute() {
    double volts = 0;
    findHome = indexer.getShootBeam();
    // home to front sensor, then back up a bit
    if(!foundHome){
      if(!findHome) volts = kVolts;
      else{
        volts = 0;
        foundHome = true;
      }
    }
    else{
      if(findHome) volts = -kVolts;
      else{
        end(false);
      }
    }

    indexer.setVolts(volts);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setVolts(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
