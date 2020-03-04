/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Shooter;

public class SetShooterState extends CommandBase {

  private final Shooter shooter;
  private final ShooterState state;
  private boolean started = false;

  public SetShooterState(Shooter shooter, ShooterState state) {
    this.shooter = shooter;
    this.state = state;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    started = true;
    shooter.getWristController().reset(shooter.getWristDegrees());
  }

  @Override
  public void execute() {
    shooter.setState(state);
  }

  @Override
  public void end(boolean interrupted) {
      started = false;
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Wrist Error", shooter.getWristController().getPositionError());
    SmartDashboard.putNumber("Shooter Error", shooter.getShooterError());
    return started && shooter.getWristController().atGoal() && shooter.checkIfStable();
  }
}
