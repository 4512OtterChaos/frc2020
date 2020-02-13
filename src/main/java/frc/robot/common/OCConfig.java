/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import static frc.robot.common.Constants.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * Used for configuring motor settings.
 */
public class OCConfig {
    public enum ConfigType{
        DRIVE(Drivetrain.kStallLimit, Drivetrain.kFreeLimit, Drivetrain.kRampRaw),
        SHOOTER(Shooter.kStallLimit, Shooter.kFreeLimit, Shooter.kRampRaw),
        SHOOTERWRIST(ShooterWrist.kStallLimit, ShooterWrist.kFreeLimit, ShooterWrist.kRampRaw),
        INTAKE(Intake.kStallLimit, Intake.kFreeLimit, Intake.kRampRaw),
        INDEXER(Indexer.kStallLimit, Indexer.kFreeLimit, Indexer.kRampRaw),
        LIFT(Lift.kStallLimit, Lift.kFreeLimit, Lift.kRampRaw);

        public final int stallLimit;
        public final int freeLimit;
        public final double rampRate;
        private ConfigType(int stallLimit, int freeLimit, double rampRate){
            this.stallLimit = stallLimit;
            this.freeLimit = freeLimit;
            this.rampRate = rampRate;
        }
    }

    public static CANSparkMax createNEO(int port){
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        return motor;
    }
    public static CANSparkMax createNEO(int port, ConfigType type){
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        configMotors(type, motor);
        return motor;
    }

    /**
     * Sets following for drivetrain motors and inverts sides.
     * @param leftMotors
     * @param rightMotors
     * @param isRightInverted
     */
    public static void configureDrivetrain(CANSparkMax[] leftMotors, CANSparkMax[] rightMotors, boolean isRightInverted){
        for(int i=0; i<leftMotors.length; i++){
            if(i>0) setFollower(leftMotors[0], false, leftMotors[i]);
            else leftMotors[i].setInverted(!isRightInverted);
        }
        for(int i=0; i<rightMotors.length; i++){
            if(i>0) setFollower(rightMotors[0], false, rightMotors[i]);
            else rightMotors[i].setInverted(isRightInverted);
        }
    }

    /**
     * Configures each given motor according to {@link ConfigType}.
     * <p>Does not invert nor set followers.
     * @param motors Array of motors
     */
    public static void configMotors(ConfigType type, CANSparkMax... motors){
        configMotors(type.stallLimit, type.freeLimit, type.rampRate, motors);
    }
    /**
     * Configures each given motor with given settings.
     * <p>Does not invert nor set followers.
     * @param motors Array of motors
     */
    public static void configMotors(int stallLimit, int freeLimit, double rampRate, CANSparkMax... motors){
        for(CANSparkMax motor : motors){
            // Make sure motor config is clean
            motor.restoreFactoryDefaults();

            // Ramp motors
            motor.setOpenLoopRampRate(rampRate);
            motor.setClosedLoopRampRate(rampRate);

            // Current limits (don't kill the motors)
            if(stallLimit!=freeLimit) motor.setSmartCurrentLimit(stallLimit, freeLimit);
            else motor.setSmartCurrentLimit(stallLimit);

            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        }
        saveConfig(motors);
    }
    /**
     * Burns configuration to flash on given motors.
     */
    public static void saveConfig(CANSparkMax... motors){
        for(CANSparkMax motor : motors){
            motor.burnFlash();
        }
    }

    /**
     * Sets idle mode of given motors.
     * @param mode IdleMode (Brake or Coast)
     * @param motors Motors to set
     */
    public static void setIdleMode(IdleMode mode, CANSparkMax... motors){
        for(CANSparkMax motor : motors) motor.setIdleMode(mode);
    }
    public static void setFollower(CANSparkMax master, boolean inverted, CANSparkMax... followers){
        for(CANSparkMax motor : followers){
            motor.follow(master, inverted);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        }
    }

}
