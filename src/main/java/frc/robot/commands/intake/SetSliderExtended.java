// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetSliderExtended extends CommandBase {

    private final Intake intake;
    private final boolean extended;

    public SetSliderExtended(Intake intake, boolean extended) {
        this.intake = intake;
        this.extended = extended;
        addRequirements(intake);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setSliderIsExtended(extended);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
