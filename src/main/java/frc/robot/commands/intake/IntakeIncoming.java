/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeIncoming extends CommandBase {
    
    private final Intake intake;
    private final BooleanSupplier ballDetector;
    private Timer unjamTimer = new Timer();
    private final double unjamTime = 0.08;
    private boolean wasBallSeen = false;

    public IntakeIncoming(Intake intake, BooleanSupplier ballDetector) {
        this.intake = intake;
        this.ballDetector = ballDetector;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        boolean seesBall = ballDetector.getAsBoolean();
        if(seesBall){
            if(!wasBallSeen){
                unjamTimer.start();
                //intake.setSliderExtended(false);
            }
            if(unjamTimer.get()<=unjamTime){
                intake.setRollerVolts(1);
                intake.setFenceVolts(-12);
            }
            else{
                unjamTimer.stop();
            }
        }
        else{
            if(wasBallSeen){
                unjamTimer.stop();
                unjamTimer.reset();
                //intake.setSliderExtended(true);
            }
            intake.setRollerVolts(7);
            intake.setFenceVolts(12);
        }


    }
    
    @Override
    public void end(boolean interrupted) {
        intake.setRollerVolts(0);
        intake.setFenceVolts(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
