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

    public static ShooterState lerp(ShooterState a, ShooterState b, double percent){
        return new ShooterState(
            MathHelp.lerp(a.angle, b.angle, percent),
            MathHelp.lerp(a.rpm, b.rpm, percent)
        );
    }
}
