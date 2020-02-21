/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.common.ShooterState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Shot Analysis Machine
 */
public class SAM implements Loggable {

    TreeMap<Double, ShooterState> shotTable = new TreeMap<Double, ShooterState>();

    @Log
    private double shotConfidence = 0; // Percentage rating of how confident we are in shooting
    @Log
    private double innerShotConfidence = 0;
    private double confidentTime = 0;

    public SAM(){
        // populate lookup table (inches, angle, rpm)
        shotTable.put(60.0, new ShooterState(40, 3500));
        shotTable.put(120.0, new ShooterState(30, 4000));
    }

    /**
     * Returns a ShooterState between two others by interpolating with percent.
     * @param percent (0-1) 0 = a, 1 = b
     */
    private static ShooterState lerpShots(double percent, ShooterState a, ShooterState b){
        double angle = MathHelp.lerp(percent, a.angle, b.angle);
        double rpm = MathHelp.lerp(percent, a.rpm, b.rpm);
        return new ShooterState(angle, rpm);
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
        return lerpShots(percent, shotTable.get(close), shotTable.get(far));
    }

    public double getShotConfidence(double targetAngle, double currAngle, ShooterState currState){
        return shotConfidence;
    }
    public double getInnerShotConfidence(double targetAngle, double currAngle, double yOffset, ShooterState currState){
        return innerShotConfidence;
    }

    public void log(){
    }
}
