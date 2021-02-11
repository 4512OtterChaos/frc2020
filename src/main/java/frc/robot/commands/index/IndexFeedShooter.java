/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.index;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.OCLedManager;
import frc.robot.common.OCLedManager.Pattern;
import frc.robot.subsystems.Indexer;

public class IndexFeedShooter extends CommandBase {
    
    private final Indexer indexer;
    private final BooleanSupplier isReady;
    private double unprimeTime = 0;
    private final double unprimeThreshold = 1; // seconds till thinks empty
    private double lastTime = 0;

    public IndexFeedShooter(Indexer indexer, BooleanSupplier isReady) {
        this.indexer = indexer;
        this.isReady = isReady;
        addRequirements(indexer);
    }
    
    @Override
    public void initialize() {
        unprimeTime = 0;
        lastTime = Timer.getFPGATimestamp();
        OCLedManager.setPattern(Pattern.YellowDash);
    }
    
    @Override
    public void execute() {
        boolean notPrimed = !indexer.getFlightBeam();
        boolean ready = isReady.getAsBoolean();
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        if(ready||notPrimed){
            indexer.setVolts(4.5, 4.5);
            OCLedManager.setPattern(Pattern.Green);
            if(notPrimed) unprimeTime += dt;
            else{
                unprimeTime = 0;
            }
        }
        else{
            indexer.setVolts(0, 0);
            OCLedManager.setPattern(Pattern.YellowDash);
        }
        lastTime = now;
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.setVolts(0, 0);
        OCLedManager.setPattern(Pattern.AutomaticWave);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return unprimeTime>unprimeThreshold;
    }
}
