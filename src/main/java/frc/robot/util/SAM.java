/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.TreeMap;

import frc.robot.common.ShooterState;

/**
 * Shot Analysis Machine
 */
public class SAM {

    TreeMap<Double, ShooterState> shotTable = new TreeMap<Double, ShooterState>();

    private double shotConfidence = 0; // Percentage rating of how confident we are in shooting
    private double innerShotConfidence = 0;
    private double confidentTime = 0;

    public SAM(){
        // populate lookup table (inches, angle, rpm)
        shotTable.put(60.0, new ShooterState(40, 3500));
        shotTable.put(120.0, new ShooterState(30, 4000));
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

    public double getShotConfidence(ShooterState targetState, ShooterState currState){
        return shotConfidence;
    }
    public double getInnerShotConfidence(ShooterState targetState, double yOffset, ShooterState currState){
        return innerShotConfidence;
    }

    public void log(){
    }
}
