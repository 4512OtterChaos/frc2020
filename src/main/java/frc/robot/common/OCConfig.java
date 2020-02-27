/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import static frc.robot.common.Constants.*;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * Used for configuring motor settings.
 */
public final class OCConfig {
    public enum ConfigType{
        NONE(0,0,0),
        DRIVE(DrivetrainConstants.kStallLimit, DrivetrainConstants.kFreeLimit, DrivetrainConstants.kRampRaw),
        SHOOTER(ShooterConstants.kStallLimit, ShooterConstants.kFreeLimit, ShooterConstants.kRampRaw),
        SHOOTERWRIST(ShooterWristConstants.kStallLimit, ShooterWristConstants.kFreeLimit, ShooterWristConstants.kRampRaw),
        INTAKE(IntakeConstants.kStallLimit, IntakeConstants.kFreeLimit, IntakeConstants.kRampRaw),
        INTAKEARM(IntakeArmConstants.kStallLimit, IntakeArmConstants.kFreeLimit, IntakeArmConstants.kRampRaw),
        INDEXER(IndexerConstants.kStallLimit, IndexerConstants.kFreeLimit, IndexerConstants.kRampRaw),
        LIFT(LiftConstants.kStallLimit, LiftConstants.kFreeLimit, LiftConstants.kRampRaw);

        public final int stallLimit;
        public final int freeLimit;
        public final double rampRate;
        private ConfigType(int stallLimit, int freeLimit, double rampRate){
            this.stallLimit = stallLimit;
            this.freeLimit = freeLimit;
            this.rampRate = rampRate;
        }
    }

    public static CANSparkMax createMAX(int port, ConfigType type){
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        configMotors(type, motor);
        return motor;
    }
    public static WPI_TalonSRX createSRX(int port, ConfigType type){
        WPI_TalonSRX motor = new WPI_TalonSRX(port);
        return motor;
    }

    /**
     * Sets following for drivetrain motors and inverts sides.
     * @param leftMotors
     * @param rightMotors
     * @param isRightInverted
     */
    public static void configureDrivetrain(CANSparkMax[] leftMotors, CANSparkMax[] rightMotors, boolean isRightInverted){
        //boolean inverted = (isRightInverted && !isInverted) || (!isRightInverted&&isInverted);

        for(int i=0; i<leftMotors.length; i++){
            if(i>0) setFollower(leftMotors[0], false, leftMotors[i]);
            else leftMotors[0].setInverted(!isRightInverted);         
        }
        for(int i=0; i<rightMotors.length; i++){
            if(i>0) setFollower(rightMotors[0], false, rightMotors[i]);
            else rightMotors[0].setInverted(isRightInverted);
        }
    }

    /**
     * Configures each given motor according to {@link ConfigType}.
     * <p>Does not invert nor set followers.
     * @param motors Array of motors
     */
    public static void configMotors(ConfigType type, CANSparkMax... motors){
        if(type!=ConfigType.NONE) configMotors(type.stallLimit, type.freeLimit, type.rampRate, motors);
    }
    /**
     * Configures each given motor according to {@link ConfigType}.
     * <p>Does not invert nor set followers.
     * @param motors Array of motors
     */
    public static void configMotors(ConfigType type, WPI_TalonSRX... motors){
        if(type!=ConfigType.NONE) configMotors(type.stallLimit, type.freeLimit, type.rampRate, motors);
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

            //motor.enableVoltageCompensation(12);

            // Current limits (don't kill the motors)
            if(stallLimit!=freeLimit) motor.setSmartCurrentLimit(stallLimit, freeLimit);
            else motor.setSmartCurrentLimit(stallLimit);
        }
        //setStatusNormal(motors);
        saveConfig(motors);
    }
    /**
     * Configures each given motor with given settings.
     * <p>Does not invert nor set followers.
     * @param motors Array of motors
     */
    public static void configMotors(int stallLimit, int freeLimit, double rampRate, WPI_TalonSRX... motors){
        for(WPI_TalonSRX motor : motors){
            // Make sure motor config is clean
            motor.configFactoryDefault();

            // Ramp motors
            motor.configOpenloopRamp(rampRate);
            motor.configClosedloopRamp(rampRate);

            // Current limits (don't kill the motors)
            motor.configContinuousCurrentLimit(freeLimit);
            motor.configPeakCurrentLimit(stallLimit);
            motor.configPeakCurrentDuration(100);
            motor.enableCurrentLimit(true);

            //motor.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        }
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
    /**
     * Sets idle mode of given motors.
     * @param mode IdleMode (Brake or Coast)
     * @param motors Motors to set
     */
    public static void setIdleMode(NeutralMode mode, WPI_TalonSRX... motors){
        for(WPI_TalonSRX motor : motors) motor.setNeutralMode(mode);
    }
    public static void setFollower(CANSparkMax master, boolean inverted, CANSparkMax... followers){
        for(CANSparkMax motor : followers){
            motor.follow(master, inverted);
        }
        setStatusSlow(followers);
    }
    public static void setFollower(WPI_TalonSRX master, boolean inverted, WPI_TalonSRX... followers){
        for(WPI_TalonSRX motor : followers){
            motor.follow(master);
            motor.setInverted(inverted ? InvertType.OpposeMaster : InvertType.FollowMaster);
            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
            motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        }
    }

    public static void setStatusFast(CANSparkMax... motors){
        for(CANSparkMax motor:motors){
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        }
    }
    public static void setStatusNormal(CANSparkMax... motors){
        for(CANSparkMax motor:motors){
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        }
    }
    public static void setStatusSlow(CANSparkMax... motors){
        for(CANSparkMax motor:motors){
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
            motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        }
    }

}
