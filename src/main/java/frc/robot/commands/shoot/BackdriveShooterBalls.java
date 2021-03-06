/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Shooter;

public class BackdriveShooterBalls extends ConditionalCommand {
    
    private final BooleanSupplier getIndexFlightBeam;

    public BackdriveShooterBalls(Shooter shooter, BooleanSupplier getIndexFlightBeam) {
        super(
            new StartEndCommand(
                ()->{
                    shooter.setShooterVelocity(-1000);
                }, 
                ()->{
                    shooter.setShooterVelocity(0);
                }, shooter).withInterrupt(()->!getIndexFlightBeam.getAsBoolean()),
            new InstantCommand(),
            getIndexFlightBeam
        );
        this.getIndexFlightBeam = getIndexFlightBeam;
        addRequirements(shooter);
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
        return !getIndexFlightBeam.getAsBoolean();
    }
}
