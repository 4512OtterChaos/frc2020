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
import frc.robot.states.ShooterState;

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
        shotTable.put(50.0, new ShooterState(64, 2300));
        shotTable.put(80.0, new ShooterState(47, 2500));
        shotTable.put(110.0, new ShooterState(42, 2500));
        shotTable.put(140.0, new ShooterState(38, 2600));
        shotTable.put(175.0, new ShooterState(35, 2600));
        shotTable.put(200.0, new ShooterState(32, 2900));
        shotTable.put(235.0, new ShooterState(30, 3000));
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
     * Returns a percentage representing how confident the system is that it can make a given shot.
     * @param targetState Desired shooter state
     * @param currState Actual shooter state
     * @param inchesDist Distance from outer port in inches
     * @param targetHeading Desired robot heading
     * @param currHeading Actual robot heading
     * @param isInnerShot Whether to check for an inner or outer port shot
     * @return Percentage(0-1) confidence
     */
    public double getShotConfidence(ShooterState targetState, ShooterState currState, double inchesDist, Rotation2d targetHeading, Rotation2d currHeading, boolean isInnerShot){
        ShooterState error = targetState.minus(currState);
        double angleError = Math.abs(error.angle);
        double rpmError = Math.abs(error.rpm);
        double headingError = Math.abs(MathHelp.getContinuousError(targetHeading.getDegrees() - currHeading.getDegrees(), 360));

        double distPercent = MathHelp.findPercentage(inchesDist, shotTable.firstKey(), shotTable.lastKey()); // At longer distances we want smaller tolerances
        final double angleTolerance = MathHelp.lerp(distPercent, 2, 1);
        final double rpmTolerance = MathHelp.lerp(distPercent, 300, 100);
        final double headingTolerance = MathHelp.lerp(distPercent, 4, 0.75);
        double angleConfidence = angleError > angleTolerance ? 0 : MathHelp.findPercentage(angleError, angleTolerance, 0);
        double rpmConfidence = rpmError > rpmTolerance ? 0 : MathHelp.findPercentage(rpmError, rpmTolerance, 0);
        double headingConfidence = headingError > headingTolerance ? 0 : MathHelp.findPercentage(headingError, headingTolerance, 0);

        double outerConfidence = angleConfidence*rpmConfidence*headingConfidence;

        // yaw = angle to inner port
        final double yawTolerance = MathHelp.lerp(distPercent, 12, 8);
        double yawMagnitude = Math.abs(currHeading.getDegrees());
        double yawConfidence = yawMagnitude > yawTolerance ? 0 : MathHelp.findPercentage(yawMagnitude, yawTolerance, 0);

        double innerConfidence = Math.pow(outerConfidence, 2)*yawConfidence;

        return isInnerShot ? innerConfidence : outerConfidence;
    }
    /**
     * Returns whether or not the system calculates the given shot to be confident and ready.
     * @param targetState Desired shooter state
     * @param currState Actual shooter state
     * @param inchesDist Distance from outer port in inches
     * @param targetHeading Desired robot heading
     * @param currHeading Actual robot heading
     * @param isInnerShot Whether to check for an inner or outer port shot
     */
    public boolean getIsReady(ShooterState targetState, ShooterState currState, double inchesDist, Rotation2d targetHeading, Rotation2d currHeading, boolean innerShot){
        boolean isConfident = getShotConfidence(targetState, currState, inchesDist, targetHeading, currHeading, innerShot) >= confidenceThreshold;
        boolean isReadyAndConfident = false; // I believe in you SAS
        double now = Timer.getFPGATimestamp();
        double dt;

        // Require a short time to pass before the system returns confident
        if(innerShot){
            dt = now - lastInnerTime;
            lastInnerTime = now;
            if(dt>0.2) dt = 0.05; // Avoid time gaps between use of this method
            if(isConfident){
                innerConfidentTime += dt;
            }
            else{
                double max = innerConfidentTime >= confidentTimeThreshold ? confidentTimeThreshold : innerConfidentTime;
                innerConfidentTime = max - dt;
            }
            isReadyAndConfident = innerConfidentTime >= confidentTimeThreshold;
        }
        else{
            dt = now - lastOuterTime;
            lastOuterTime = now;
            if(dt>0.2) dt = 0.05; // Avoid time gaps between use of this method
            if(isConfident){
                outerConfidentTime += dt;
            }
            else{
                double max = outerConfidentTime >= confidentTimeThreshold ? confidentTimeThreshold : outerConfidentTime;
                outerConfidentTime = max - dt;
            }
            isReadyAndConfident = outerConfidentTime >= confidentTimeThreshold;
        }

        return isReadyAndConfident;
    }

    public void log(){
    }
}
