/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeDown;
import frc.robot.commands.shoot.SetShooterState;
import frc.robot.common.Constants.ShooterWristConstants;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SafeIntakeDown extends SequentialCommandGroup {
  /**
   * Creates a new SafeIntakeDown.
   */
  public SafeIntakeDown(Intake intake, Shooter shooter) {
    super(
        new ConditionalCommand(
            new SetShooterState(shooter, new ShooterState(ShooterWristConstants.kClearIntakeDegrees, shooter.getRPM())),
            new InstantCommand(),
            ()->shooter.getWristDegrees()<ShooterWristConstants.kClearIntakeDegrees
        ),
        new IntakeDown(intake)
    );
  }
}
