/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import frc.robot.util.MathHelp;

public class ShooterState {
    public final double angle;
    public final double rpm;

    public ShooterState(double angle, double rpm){
        this.angle = angle;
        this.rpm = rpm;
    }

    /**
     * Returns the difference between this state and another.
     */
    public ShooterState minus(ShooterState other){
        return new ShooterState(angle-other.angle, rpm-other.rpm);
    }

    /**
     * Returns the result of linearly interpolating between two states based on percent.
     */
    public static ShooterState lerp(double percent, ShooterState a, ShooterState b){
        return new ShooterState(
            MathHelp.lerp(percent, a.angle, b.angle),
            MathHelp.lerp(percent, a.rpm, b.rpm)
        );
    }
}
