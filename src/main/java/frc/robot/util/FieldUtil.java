/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Helper methods for field dimensions / transforms.
 */
public final class FieldUtil {

    /**
     * Returns pose with (x,y) based off of heading and distance.
     * @param heading Object heading on field
     * @param distMeters Distance in meters the object is from the target
     */
    public static Pose2d getRelativePose(Rotation2d heading, double distMeters){
        double radians = heading.getRadians()+Math.PI*0.5;
        double y = Math.cos(radians)*distMeters;
        double x = Math.sin(radians)*-distMeters;
        return new Pose2d(x, y, heading);
    }

    /**
     * Returns the angle between the object and the target.
     * Keep in mind angles are counter-clockwise positive and 0 degrees is the x-axis.
     * E.g. the absolute angle an object would turn to to face the target
     */
    public static Rotation2d getRelativeAngle(Translation2d objectTran, Translation2d targetTran){
        double x = targetTran.getX() - objectTran.getX();
        double y = targetTran.getY() - objectTran.getY();
        return new Rotation2d(Math.atan2(y, x)); // this works because x is (relatively) forward in FRC
    }
    /**
     * Returns the difference between the current object rotation to the angle between the object and target.
     * E.g. adding this rotation to an object would cause it to face the target
     */
    public static Rotation2d getRelativeHeading(Pose2d objectPose, Translation2d targetTran){
        Rotation2d theta = getRelativeAngle(objectPose.getTranslation(), targetTran);
        return theta.minus(objectPose.getRotation());
    }
}
