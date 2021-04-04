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
import frc.robot.common.LEDPattern;
import frc.robot.common.OCLEDManager;
import frc.robot.common.OCLEDManager.PatternCard;
import frc.robot.subsystems.Indexer;

public class IndexFeedShooter extends CommandBase {
    
    private final Indexer indexer;
    private final BooleanSupplier isReady;
    private double indexVolts = 3.5;

    private OCLEDManager manager;
    private PatternCard shootCard;
    private LEDPattern shootPattern;

    public IndexFeedShooter(Indexer indexer, BooleanSupplier isReady) {
        this.indexer = indexer;
        this.isReady = isReady;
        addRequirements(indexer);
    }
    public IndexFeedShooter(Indexer indexer, BooleanSupplier isReady, double indexVolts) {
        this.indexer = indexer;
        this.isReady = isReady;
        this.indexVolts = indexVolts;
        addRequirements(indexer);
    }
    
    @Override
    public void initialize() {
        manager = OCLEDManager.getInstance();
        LEDPattern primePattern = new LEDPattern(manager).presetFlashing(LEDPattern.kYellowHue, 255, 255, 12);
        shootCard = manager.new PatternCard(1);
        LEDPattern readyPattern = new LEDPattern(manager).presetSolid(LEDPattern.kGreenHue, 255, 255);
        shootPattern = LEDPattern.conditionalWith(primePattern, readyPattern, isReady);
        manager.addPattern(shootCard, shootPattern);
    }
    
    @Override
    public void execute() {
        if(isReady.getAsBoolean()){
            indexer.setVolts(indexVolts);
        }
        else{
            indexer.setVolts(0);
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.setVolts(0);
        shootCard.end();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
