/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
* Shot Analysis System
*/
public class SAS {
    
    TreeMap<Double, ShooterState> shotTable = new TreeMap<Double, ShooterState>();
    
    private double outerConfidentTime = 0;
    private double lastOuterTime = Timer.getFPGATimestamp();
    private double innerConfidentTime = 0;
    private double lastInnerTime = Timer.getFPGATimestamp();
    private final double confidentTimeThreshold = 0.3;
    
    private final double confidenceThreshold = 0.8;
    
    
    public SAS(){
        // populate lookup table (inches, angle, rpm)
        shotTable.put(50.0, new ShooterState(64, 2000));
        shotTable.put(80.0, new ShooterState(50, 2100));
        shotTable.put(110.0, new ShooterState(45, 2200));
        shotTable.put(140.0, new ShooterState(38, 2400));
        shotTable.put(175.0, new ShooterState(35, 2500));
        shotTable.put(200.0, new ShooterState(32, 2800));
        shotTable.put(235.0, new ShooterState(30, 2900));
    }
    
    /**
    * Returns a {@link ShooterState} at inchesDist from the outer port.
    */
    public ShooterState findShot(double inchesDist){
        double minDist = shotTable.firstKey();
        double maxDist = shotTable.lastKey();
        inchesDist = MathHelp.clamp(inchesDist, minDist, maxDist);
        Double close = shotTable.floorKey(inchesDist);
        Double far = shotTable.ceilingKey(inchesDist);
        double boundDiff = far - close;
        double actualDiff = inchesDist - close;
        double percent;
        if(boundDiff != 0.0){
            percent = actualDiff / boundDiff;
        }
        else{
            percent = 0;
        }
        return ShooterState.lerp(percent, shotTable.get(close), shotTable.get(far));
    }
    
    /**
     * Percentage confidence in current shooter angle based on target angle based on distance
     * @param distance Inches dist from powerport
     */
    public double getAngleConfidence(double distance, Shooter shooter){
        double distPercent = MathHelp.findPercentage(distance, shotTable.firstKey(), shotTable.lastKey()); // At longer distances we want smaller tolerances
        double angleTolerance = MathHelp.lerp(distPercent, 2, 1.5);
        
        ShooterState error = shooter.getTargetState().minus(shooter.getCurrentState());
        double angleError = Math.abs(error.angle);

        double angleConfidence = angleError > angleTolerance ? 0 : MathHelp.findPercentage(angleError, angleTolerance, 0);
        SmartDashboard.putNumber("Confidence Angle", angleConfidence);
        return angleConfidence;
    }
    /**
     * Percentage confidence in current RPM versus target RPM based on distance
     * @param distance Inches dist from powerport
     */
    public double getRPMConfidence(double distance, Shooter shooter){
        double distPercent = MathHelp.findPercentage(distance, shotTable.firstKey(), shotTable.lastKey()); // At longer distances we want smaller tolerances
        double rpmTolerance = MathHelp.lerp(distPercent, 300, 100);
        
        ShooterState error = shooter.getTargetState().minus(shooter.getCurrentState());
        double rpmError = Math.abs(error.rpm);

        double rpmConfidence = rpmError > rpmTolerance ? 0 : MathHelp.findPercentage(rpmError, rpmTolerance, 0);
        SmartDashboard.putNumber("Confidence RPM", rpmConfidence);
        return rpmConfidence;
    }
    /**
     * Percentage confidence in current heading versus target heading(uses TurnTo if available)
     * @param distance Inches dist from powerport
     */
    public double getHeadingConfidence(double distance, Drivetrain drivetrain){
        double distPercent = MathHelp.findPercentage(distance, shotTable.firstKey(), shotTable.lastKey()); // At longer distances we want smaller tolerances
        double headingTolerance = MathHelp.lerp(distPercent, 3, 1.5);

        Rotation2d currHeading = drivetrain.getHeading();
        Rotation2d targetHeading = drivetrain.getTurnToTarget() != null ? drivetrain.getTurnToTarget() : currHeading;
        double headingError = Math.abs(MathHelp.getContinuousError(targetHeading.getDegrees() - currHeading.getDegrees(), 360));
        
        double headingConfidence = headingError > headingTolerance ? 0 : MathHelp.findPercentage(headingError, headingTolerance, 0);
        SmartDashboard.putNumber("Confidence Heading", headingConfidence);
        return headingConfidence;
    }

    /**
     * Returns if confident in all variables and ready to shoot
     * @param distance
     */
    public boolean getIsReady(double distance, Shooter shooter, Drivetrain drivetrain){
        boolean isConfident = MathHelp.min(
            getAngleConfidence(distance, shooter),
            getRPMConfidence(distance, shooter),
            getHeadingConfidence(distance, drivetrain)
        ) > confidenceThreshold;
        
        double now = Timer.getFPGATimestamp();
        
        // Require a short time to pass before the system returns confident
        boolean isReadyAndConfident = (now - shooter.getLastShotTime() > confidentTimeThreshold) && isConfident; // I believe in you SAS
        
        return isReadyAndConfident;
    }
    
    public void log(){
    }
}
