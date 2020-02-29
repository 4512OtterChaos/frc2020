/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.common.Constants.IntakeArmConstants;
import frc.robot.states.IntakeState;
import frc.robot.subsystems.Intake;

public class IntakeUpClear extends SequentialCommandGroup {

  public IntakeUpClear(Intake intake) {
    super(
      new InstantCommand(()->intake.setSliderExtended(false),intake),
      new WaitCommand(0.2),
      new SetIntakeState(intake, new IntakeState(IntakeArmConstants.kHigherSafeDegrees, false))
    );
  }
}
