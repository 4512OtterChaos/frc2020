/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.SAS;

public class AimOuter extends CommandBase {

  final Shooter shooter;
  final Limelight limelight;
  final SAS sas;

  public AimOuter(Shooter shooter, Limelight limelight, SAS sas) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.sas = sas;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // adjust shooter depending on target distance
    shooter.setState(sas.findShot(limelight.getTrigDistance()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setState(ShooterState.kIdleState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
