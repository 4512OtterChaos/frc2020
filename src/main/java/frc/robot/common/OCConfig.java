/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import static frc.robot.common.Constants.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Used for configuring motor settings.
 */
public class OCConfig {

    /**
     * Configures each motor given with drive settings.
     * Sets followers and inverts.
     * @param isRightInverted Is right or left inverted
     */
    public static void configDriveMotors(CANSparkMax[] leftMotors, CANSparkMax[] rightMotors, boolean isRightInverted){
        configMotors(leftMotors);
        configMotors(rightMotors);

        for(int i=0;i<leftMotors.length;i++){
            if(i>0) leftMotors[i].follow(leftMotors[0]);
            else leftMotors[i].setInverted(!isRightInverted);
        }
        for(int i=0;i<rightMotors.length;i++){
            if(i>0) rightMotors[i].follow(rightMotors[0]);
            else rightMotors[i].setInverted(isRightInverted);
        }

        saveConfig(leftMotors);
        saveConfig(rightMotors);
    }

    /**
     * Configures each motor given with drive settings.
     * <p>Does not invert nor set followers.
     * @param motors Array of drive motors
     */
    public static void configMotors(CANSparkMax... motors){
        configMotors(kStallLimit, kFreeLimit, motors);
    }
    /**
     * Configures each motor given with default settings.
     * <p>Does not invert nor set followers.
     * @param motors Array of drive motors
     */
    public static void configMotors(int stallLimit, int freeLimit, CANSparkMax... motors){
        for(CANSparkMax motor:motors){
            // Make sure motor config is clean
            motor.restoreFactoryDefaults();

            // Ramp motors
            motor.setOpenLoopRampRate(kRampRaw);
            motor.setClosedLoopRampRate(kRampRaw);

            // Current limits (don't kill the motors)
            if(stallLimit!=freeLimit) motor.setSmartCurrentLimit(stallLimit, freeLimit);
            else motor.setSmartCurrentLimit(stallLimit);
        }
    }
    /**
     * Burns configuration to flash on given motors.
     */
    public static void saveConfig(CANSparkMax... motors){
        for(CANSparkMax  motor:motors){
            motor.burnFlash();
        }
    }

    /**
     * Sets idle mode of given motors.
     * @param mode IdleMode (Brake or Coast)
     * @param motors Motors to set
     */
    public static void setIdleMode(IdleMode mode, CANSparkMax... motors){
        for(CANSparkMax motor:motors) motor.setIdleMode(mode);
    }

}
