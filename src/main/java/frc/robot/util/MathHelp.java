/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import java.util.Arrays;
import java.util.Collections;

/**
* Class to store simple helper functions for calculations.
*/
public final class MathHelp {
    
    // min/max with more inputs
    public static int min(Integer... values){
        return Collections.min(Arrays.asList(values));
    }
    public static double min(Double... values){
        return Collections.min(Arrays.asList(values));
    }
    public static int max(Integer... values){
        return Collections.max(Arrays.asList(values));
    }
    public static double max(Double... values){
        return Collections.max(Arrays.asList(values));
    }
    
    // Steal clamp from MathUtil to reduce scope
    /**
    * Returns value clamped between boundaries.
    *
    * @param value Value to clamp.
    * @param a  The first boundary which clamps value.
    * @param b  The second boundary which clamps value.
    */
    public static int clamp(int value, int a, int b) {
        int low = Math.min(a,b);
        int high = Math.max(a,b);
        return Math.max(low, Math.min(value, high));
    }
    /**
    * Returns value clamped between boundaries.
    *
    * @param value Value to clamp.
    * @param a  The first boundary which clamps value.
    * @param b  The second boundary which clamps value.
    */
    public static double clamp(double value, double a, double b) {
        double low = Math.min(a,b);
        double high = Math.max(a,b);
        return Math.max(low, Math.min(value, high));
    }
    
    /**
    * Returns value rounded to closest boundary.
    *
    * @param value Value to round.
    * @param a  The first boundary to round value to.
    * @param b  The second boundary to round value to.
    */
    public static double roundToBounds(int value, int a, int b){
        value = clamp(value, a, b);
        double average = (a+b)/2.0;
        int low = Math.min(a,b);
        int high = Math.max(a,b);
        return (value >= average ? high : low);
    }
    /**
    * Returns value rounded to closest boundary.
    *
    * @param value Value to round.
    * @param a  The first boundary to round value to.
    * @param b  The second boundary to round value to.
    */
    public static double roundToBounds(double value, double a, double b){
        value = clamp(value, a, b);
        double average = (a+b)/2.0;
        double low = Math.min(a,b);
        double high = Math.max(a,b);
        return (value >= average ? high : low);
    }
    
    public static boolean isBetweenBounds(double value, double a, double b){
        double low = Math.min(a,b);
        double high = Math.max(a,b);
        return (value >= low && value <= high);
    }
    public static boolean isBetweenBounds(int value, int a, int b){
        int low = Math.min(a,b);
        int high = Math.max(a,b);
        return (value >= low && value <= high);
    }
    
    /**
    * Linearly interpolates between 'from' and 'to' by percent amount.
    * @param percent (0-1): 0 = from, 1 = to
    */
    public static double lerp(double percent, double from, double to){
        percent = clamp(percent, 0, 1);
        return (from+(to-from)*percent);
    }
    /**
    * Counterpart to lerp. Returns the percentage from 'from' to 'to' given value.
    * @param value Value between from and to
    */
    public static double findPercentage(double value, double from, double to){
        value = clamp(value, from, to);
        return (from - value) / (from - to);
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
    
    /**
    * Finds the average of an array of doubles.
    */
    public static double findAverage(double[] values){
        if(values.length==0) return 0;
        double average = 0;
        for(double value : values){
            average += value; // sum
        }
        return average / values.length;
    }
    /**
    * Finds the median of an array of doubles.
    */
    public static double findMedian(double[] values){
        int len = values.length;
        if(len==0) return 0;
        Arrays.sort(values);
        if(len%2==0) return findAverage(new double[]{values[len/2], values[len/2 - 1]});
        else return values[len/2];
    }
    
    /**
    * Returns the given array of doubles with the farthest outlier removed.
    * Outliers are found by the largest distance to the median of the array.
    */
    public static double[] trimOutlier(double[] values){
        if(values.length<=2) return values;
        
        double median = findMedian(values);
        
        int outlierIndex = 0;
        double largestDistance = 0;
        for(int i=0;i<values.length;i++){
            double distance = Math.abs(median - values[i]);
            if(distance > largestDistance){
                outlierIndex = i;
                largestDistance = distance;
            }
        }
        
        double[] trimmedValues = new double[values.length-1];
        boolean passed = false;
        for(int i=0;i<values.length;i++){
            if(i!=outlierIndex){
                if(!passed) trimmedValues[i] = values[i];
                else trimmedValues[i-1] = values[i];
            }
            else{
                passed = true;
            }
        }
        return trimmedValues;
    }
    /**
    * Returns the given array with outlierCount outliers removed.
    * Outliers are found by the largest distance to the median of the array.
    */
    public static double[] trimOutliers(double[] values, int outlierCount){
        double[] result = values;
        for(int i=0;i<outlierCount;i++){ // trimming too many times will just return values[]
            result = trimOutlier(result);
        }
        return result;
    }
    /**
    * Finds the average of given array of doubles with outliers removed.
    * @param outlierCount the amount of outliers to remove from the array
    */
    public static double getTrimmedAverage(double[] values, int outlierCount){
        return findAverage(trimOutliers(values, outlierCount));
    }
}
