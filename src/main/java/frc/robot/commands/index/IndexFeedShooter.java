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

    private OCLEDManager manager;
    private PatternCard primeCard;
    private LEDPattern primePattern;
    private PatternCard shootCard;
    private LEDPattern shootPattern;

    public IndexFeedShooter(Indexer indexer, BooleanSupplier isReady) {
        this.indexer = indexer;
        this.isReady = isReady;
        addRequirements(indexer);
    }
    
    @Override
    public void initialize() {
        manager = OCLEDManager.getInstance();
        primeCard = manager.new PatternCard(isReady, 1);
        primePattern = new LEDPattern(manager).presetFlashing(LEDPattern.kYellowHue, 255, 255, 20);
        shootCard = manager.new PatternCard(()->!isReady.getAsBoolean(), 1);
        shootPattern = new LEDPattern(manager).presetSolid(LEDPattern.kGreenHue, 255, 255);
    }
    
    @Override
    public void execute() {
        if(isReady.getAsBoolean()){
            indexer.setVolts(4);
            manager.addPattern(shootCard, shootPattern);
        }
        else{
            indexer.setVolts(0);
            manager.addPattern(primeCard, primePattern);
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.setVolts(0);
        primeCard.end();
        shootCard.end();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
