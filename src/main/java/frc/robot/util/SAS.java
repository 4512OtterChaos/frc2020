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
    private final double confidentTimeThreshold = 0.4;
    
    private final double confidenceThreshold = 0.7;
    
    
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
    * Returns a {@link ShooterState} at distanceInches from the outer port.
    */
    public ShooterState findShot(double distanceInches){
        double minDist = shotTable.firstKey();
        double maxDist = shotTable.lastKey();
        distanceInches = MathHelp.clamp(distanceInches, minDist, maxDist);
        Double close = shotTable.floorKey(distanceInches);
        Double far = shotTable.ceilingKey(distanceInches);
        double boundDiff = far - close;
        double actualDiff = distanceInches - close;
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
     * @param distanceInches Inches dist from powerport
     */
    public double getAngleConfidence(double distanceInches, Shooter shooter){
        double distPercent = MathHelp.findPercentage(distanceInches, shotTable.firstKey(), shotTable.lastKey()); // At longer distances we want smaller tolerances
        double angleTolerance = MathHelp.lerp(distPercent, 3, 2);
        
        ShooterState error = shooter.getTargetState().minus(shooter.getCurrentState());
        double angleError = Math.abs(error.angle);

        double angleConfidence = angleError > angleTolerance ? 0 : MathHelp.findPercentage(angleError, angleTolerance, 0);
        SmartDashboard.putNumber("Confidence Angle", angleConfidence);
        return angleConfidence;
    }
    /**
     * Percentage confidence in current RPM versus target RPM based on distance
     * @param distanceInches Inches dist from powerport
     */
    public double getRPMConfidence(double distanceInches, Shooter shooter){
        double distPercent = MathHelp.findPercentage(distanceInches, shotTable.firstKey(), shotTable.lastKey()); // At longer distances we want smaller tolerances
        double rpmTolerance = MathHelp.lerp(distPercent, 300, 100);
        
        ShooterState error = shooter.getTargetState().minus(shooter.getCurrentState());
        double rpmError = Math.abs(error.rpm);

        double rpmConfidence = rpmError > rpmTolerance ? 0 : MathHelp.findPercentage(rpmError, rpmTolerance, 0);
        SmartDashboard.putNumber("Confidence RPM", rpmConfidence);
        return rpmConfidence;
    }
    /**
     * Percentage confidence in current heading versus target heading(uses TurnTo if available)
     * @param distanceInches Inches dist from powerport
     */
    public double getHeadingConfidence(double distanceInches, Drivetrain drivetrain){
        double distPercent = MathHelp.findPercentage(distanceInches, shotTable.firstKey(), shotTable.lastKey()); // At longer distances we want smaller tolerances
        double headingTolerance = MathHelp.lerp(distPercent, 7, 5);

        double headingError = Math.abs(drivetrain.getTurnToError().getDegrees());
        
        double headingConfidence = headingError > headingTolerance ? 0 : MathHelp.findPercentage(headingError, headingTolerance, 0);
        SmartDashboard.putNumber("Confidence Heading", headingConfidence);
        return headingConfidence;
    }

    /**
     * Returns if confident in all shot variables
     * @param distanceInches
     */
    public boolean getIsConfident(double distanceInches, Shooter shooter, Drivetrain drivetrain){
        boolean isConfident = MathHelp.min(
            getAngleConfidence(distanceInches, shooter),
            getRPMConfidence(distanceInches, shooter),
            getHeadingConfidence(distanceInches, drivetrain)
        ) > confidenceThreshold;
                
        return isConfident;
    }
    /**
     * Returns if confident in all shot variables and is ready to shoot(cooldown between shots)
     * @param distanceInches
     */
    public boolean getIsReady(double distanceInches, Shooter shooter, Drivetrain drivetrain){
        double now = Timer.getFPGATimestamp();
        // Require a short time to pass before the system returns confident
        boolean isReady = (now - shooter.getLastShotTime() > confidentTimeThreshold);

        return getIsConfident(distanceInches, shooter, drivetrain) && isReady; // I believe in you SAS
    }
    
    public void log(){
    }
}
