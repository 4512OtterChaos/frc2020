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

  boolean foundHome = false;
  double homeTime = 0;
  /**
   * Creates a new IndexIncoming.
   */
  public IndexHomeShooter(Indexer indexer) {
    this.indexer = indexer;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double volts = 0;
    foundHome = indexer.getFlightBeam();
    // home to front sensor, then back up a bit
    if(homeTime==0){
      if(!foundHome) volts = kVolts;
      else{
        volts = 0;
        homeTime = Timer.getFPGATimestamp();
      }
    }
    else{
      if(foundHome) volts = -kVolts;
      else{
        end(false);
      }
    }

    indexer.setVolts(volts, volts);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setVolts(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
