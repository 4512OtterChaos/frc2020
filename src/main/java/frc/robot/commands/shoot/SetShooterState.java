/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Shooter;

public class SetShooterState extends CommandBase {

  private final Shooter shooter;
  private final ShooterState state;
  private boolean started = false;
  private double lastTime = 0;
  private double leftTime = 0;
  private double rightTime = 0;
  private final double timeThreshold = 0.3;
  private final double rpmTolerance = 50;

  public SetShooterState(Shooter shooter, ShooterState state) {
    this.shooter = shooter;
    this.state = state;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    started = true;
    lastTime = Timer.getFPGATimestamp();
    shooter.getWristController().reset(shooter.getWristDegrees());
  }

  @Override
  public void execute() {
    shooter.setState(state);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // when wheels are at desired speed, increase time
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    double leftError = Math.abs(state.rpm - shooter.getLeftRPM());
    double rightError = Math.abs(state.rpm - shooter.getRightRPM());
    if(leftError <= rpmTolerance) leftTime += dt;
    if(rightError <= rpmTolerance) rightTime += dt;
    lastTime = now;

    // if we've had desired speed long enough, we're good
    boolean leftStable = leftTime >= timeThreshold;
    boolean rightStable = rightTime >= timeThreshold;
    return started && shooter.getWristController().atGoal() && leftStable && rightStable;
  }
}
