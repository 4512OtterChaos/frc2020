/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.IntakeState;
import frc.robot.subsystems.Intake;

public class SetIntakeState extends CommandBase {

  private final Intake intake;
  private final IntakeState state;
  private boolean started = false;

  public SetIntakeState(Intake intake, IntakeState state) {
    this.intake = intake;
    this.state = state;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    started = true;
    intake.getController().reset(intake.getArmDegrees());
    intake.setState(state);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
      intake.setArmVolts(0);
  }

  @Override
  public boolean isFinished() {
    return started && intake.getController().atGoal() && intake.getSliderExtended()==state.extended;
  }
}
