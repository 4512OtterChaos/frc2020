/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.states;

import static frc.robot.common.Constants.IntakeArmConstants.*;

/**
 * State representing intake angle and slider extension.
 */
public class IntakeState {
    public final double angle;
    public final boolean extended;
    public static final IntakeState kDownIdle = new IntakeState(0.5, false);
    public static final IntakeState kUpIdle = new IntakeState(kHigherSafeDegrees, false);

    public IntakeState(double angle, boolean extended){
        this.angle = angle;
        this.extended = extended;
    }
}
