/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexIncoming extends CommandBase {
  
  final Indexer indexer;
  /**
   * Creates a new IndexIncoming.
   */
  public IndexIncoming(Indexer indexer) {
    this.indexer = indexer;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(indexer.getFrontBeam()){
      indexer.setVolts(2, 2);
    }
    else{
      indexer.setVolts(0,0);
    }
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
