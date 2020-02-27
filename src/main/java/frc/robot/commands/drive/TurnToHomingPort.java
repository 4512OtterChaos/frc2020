/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import frc.robot.common.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldUtil;

public class TurnToHomingPort extends TurnTo {

    public TurnToHomingPort(Drivetrain drivetrain) {
      super(
          drivetrain,
          FieldUtil.getRelativeHeading(drivetrain.getOdometry().getPoseMeters(), VisionConstants.kTargetTranslation).getDegrees()
      );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
