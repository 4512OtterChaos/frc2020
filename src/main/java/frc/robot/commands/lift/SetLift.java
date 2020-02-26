/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift;

public class SetLift extends CommandBase {
    
    private final Lift lift;
    private final double rotations;
    private boolean started = false;

    public SetLift(Lift lift, double rotations) {
        this.lift = lift;
        this.rotations = rotations;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lift.setPID(rotations);
        started = true;
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        lift.setVolts(0);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return started && lift.getController().atGoal();
    }
}
