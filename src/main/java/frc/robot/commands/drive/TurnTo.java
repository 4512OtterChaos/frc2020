/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.common.Constants.DrivetrainConstants.*;

public class TurnTo extends ProfiledPIDCommand {

    private static final double kCruiseVelocityDegrees = Units.radiansToDegrees(kMaxVelocityRadians)*0.8;

  public TurnTo(Drivetrain drivetrain, double target) {
    super(
        new ProfiledPIDController(
            0, 0, 0,
            new TrapezoidProfile.Constraints(kCruiseVelocityDegrees, kCruiseVelocityDegrees*1.75)),
        () -> drivetrain.getHeading().getDegrees(),
        () -> target,
        (output, setpoint) -> {
          drivetrain.setChassisSpeedPID(0, output);
        });
    addRequirements(drivetrain);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
