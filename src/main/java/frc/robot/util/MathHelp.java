/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Class to store simple helper functions for calculations.
 */
public final class MathHelp {
    
    // Steal clamp from MathUtil to reduce scope
    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     */
    public static int clamp(int value, int low, int high) {
      return Math.max(low, Math.min(value, high));
    }
    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     */
    public static double clamp(double value, double low, double high) {
      return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns value rounded to either the low or high boundary.
     *
     * @param value Value to round.
     * @param low   The lower boundary to which to round value.
     * @param high  The higher boundary to which to round value.
     */
    public static double roundToBounds(double value, double low, double high){
      value = clamp(value, low, high);
      double average = (low+high)/2.0;
      return (value >= average ? high : low);
    }
    public static int roundToBounds(int value, int low, int high){
      value = clamp(value, low, high);
      double average = (low+high)/2.0;
      return (value >= average ? high : low);
    }

    public static boolean isBetweenBounds(double value, double low, double high){
      return (value >= low && value <= high);
    }
    public static boolean isBetweenBounds(int value, int low, int high){
      return (value >= low && value <= high);
    }

    /**
     * Linearly interpolates between a and b by percent amount.
     * @param percent (0-1): 0 = a, 1 = b
     */
    public static double lerp(double percent, double a, double b){
        percent = clamp(percent, 0, 1);
        return (a+(b-a)*percent);
    }
    /**
     * Counterpart to lerp. Returns the percentage from a to b given value.
     * @param value Value between a and b
     */
    public static double findPercentage(double value, double a, double b){
      value = clamp(value, a, b);
      return (a - value) / (a - b);
    }

    public static double getContinuousError(double error, double range){
      if(range > 0){
        error %= range;
        if (Math.abs(error) > range / 2) {
          if (error > 0) {
            return error - range;
          } else {
            return error + range;
          }
        }
      }
      return error;
    }
}
