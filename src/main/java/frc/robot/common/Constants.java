/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static final double kRobotDelta = 0.01;

    public static final double kFieldWidth = Units.inchesToMeters(323.75);
    public static final double kFieldDepth = Units.inchesToMeters(629.25);
    
    // All distance measurements should be in meters when being used
    
    public static class DrivetrainConstants{
        public static final double kTrackWidthMeters = Units.inchesToMeters(23); // Distance between center of drivetrain sides
        public static final double kWheelRadiusMeters = Units.inchesToMeters(3);

        public static final double kMaxVelocityMeters = Units.feetToMeters(14);
        public static final double kMaxVelocityRadians = Units.degreesToRadians(450);
        
        public static final double kGearRatio = 8.8888; // Motor rotations per wheel rotation

        public static final int kFreeLimit = 40;
        public static final int kStallLimit = 75;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)
        
        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared
        
        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }

    public static class ShooterConstants{
        public static final int kFreeLimit = 40;
        public static final int kStallLimit = 80;

        public static final double kRampRaw = 0.04; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }
    public static class ShooterWristConstants{
        public static final int kFreeLimit = 25;
        public static final int kStallLimit = 30;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }

    public static class IntakeConstants{
        public static final int kFreeLimit = 22;
        public static final int kStallLimit = 27;

        public static final double kRampRaw = 0.5; // seconds to full output(on the motor)
    }
    public static class IntakeArmConstants{
        public static final int kFreeLimit = 25;
        public static final int kStallLimit = 32;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;

        // Constraints
        public static final int kMaxForwardRotations = 120; // Max motor rotations
        public static final int kVelocityConstraint = 9000; // Motor RPM
        public static final int kAccelerationConstraint = 15000; // Motor RPM per second
    }

    public static class IndexerConstants{
        public static final int kFreeLimit = 22;
        public static final int kStallLimit = 27;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;
    }

    public static class LiftConstants{
        public static final double kGearRatio = 8.8888; // Motor rotations per wheel rotation

        public static final int kFreeLimit = 27;
        public static final int kStallLimit = 35;

        public static final double kRampRaw = 0.08; // seconds to full output(on the motor)

        public static final double kStaticFF = 0; // volts (Given from the characterization tool)
        public static final double kLiftWeightCounter = 0;
        public static final double kRobotWeightCounterFF = 0;
        public static final double kVelocityFF = 0; // per meters per second
        public static final double kAccelerationFF = 0; // per meters per second squared

        public static final double kP = 0; // PID Gains (For one meter/second of error, kP volts are applied)
        public static final double kI = 0; // Only P should be used for velocity control
        public static final double kD = 0;

        // Constraints
        public static final int kMaxHeightRotations = 100; // Max height in motor rotations
        public static final int kVelocityConstraint = 7000; // Motor RPM
        public static final int kAccelerationConstraint = 10000; // Motor RPM per second
    }

    public static class VisionConstants{
        public static final double kCameraAngle = 30; // Angle of camera offset from horizontal
        public static final double kCameraHeight = 24;
        public static final double kTargetHeight = 98.25;
        public static final double kLatencyMs = 11; // Image capture latency
        public static final Translation2d kCameraTranslation = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
        public static final Translation2d kTargetTranslation = new Translation2d(kFieldDepth, Units.inchesToMeters(94.66));
    }
    
    public static class AutonomousConstants{
        public static final double kMaxVelocityMeters = Units.feetToMeters(10);
        public static final double kMaxAccelerationMeters = Units.feetToMeters(6);
        public static final double kMaxCentripetalAccelerationMeters = Units.feetToMeters(3.4); // Turning acceleration given radius
    }
}
