/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.index;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexIncoming extends CommandBase {
  
  final Indexer indexer;
  final BooleanSupplier isSafe;
  /**
   * Creates a new IndexIncoming.
   */
  public IndexIncoming(Indexer indexer) {
    this.indexer = indexer;
    this.isSafe = null;

    addRequirements(indexer);
  }
  public IndexIncoming(Indexer indexer, BooleanSupplier isSafe){
      this.indexer = indexer;
      this.isSafe = isSafe;

      addRequirements(indexer);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(indexer.getFrontBeam()&&isSafe.getAsBoolean()){
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
