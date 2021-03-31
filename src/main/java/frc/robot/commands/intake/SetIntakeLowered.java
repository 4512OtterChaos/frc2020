// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

/**
 * Sets the intake up or down.
 */
public class SetIntakeLowered extends SequentialCommandGroup{
    public SetIntakeLowered(Intake intake, boolean down) {
        super(
            new ConditionalCommand(
                new WaitCommand(0.05),
                new InstantCommand(()->intake.setSliderIsExtended(true), intake)
                    .andThen(new WaitCommand(0.5)),
                intake::getSliderIsExtended
            ),
            new ConditionalCommand(
                new WaitCommand(0.05),
                new InstantCommand(()->intake.setArmIsExtended(down), intake)
                    .andThen(new WaitCommand(0.5)),
                ()->down==intake.getArmIsExtended()
            )
        );
    }
}
