/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.MathHelp;

/**
 * Custom {@link XboxController} wrapper to add convenience features for driving.
 */
public class OCXboxController extends XboxController{

    public enum DriveMode{
        ARCADE,
        TANKVOLTS,
        CURVATURE,
        CURVATUREVOLTS,
        HENRYDRIVEGAS,
        HENRYDRIVEBRAKE
    }
    private DriveMode mode;

    private static final double kDeadband = 0.12;
    private static final double kArcadeThreshold = 0.2;

    
    public static final double kSpeedDefault = 0.4;
    public static final double kSpeedFast = 0.6;
    public static final double kSpeedMax = 0.8;
    private double drivespeed = kSpeedDefault;
    
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.5);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter curveLimiter = new SlewRateLimiter(5);
    private SlewRateLimiter drivespeedLimiter = new SlewRateLimiter(2);

    /**
     * Constructs XboxController on DS joystick port.
     */
    public OCXboxController(int port){
        super(port);
    }

    public DriveMode getMode(){
        return mode;
    }
    public void setDriveMode(DriveMode mode){
        this.mode = mode;
    }

    public static double deadband(double value){
        return Math.abs(value) > kDeadband ? value : 0;
    }

    /**
     * Deadbands a value, re-scales it, and applies a power.
     * @param value Value to adjust
     * @return -1 to 1
     */
    public static double scaledPowerDeadband(double value){
        return scaledPowerDeadband(value, 1);
    }
    /**
     * Deadbands a value, re-scales it, and applies a power.
     * @param value Value to adjust
     * @param dead Deadband threshold
     * @return -1 to 1
     */
    public static double scaledPowerDeadband(double value, double exp){
        final double dead = kDeadband;
        if(Math.abs(value) < dead) return 0;
        double scale = 1.0 / (1 - dead);
        double sign = Math.copySign(1.0, value);
        value = Math.abs(value);
        double fixedValue = sign*(scale*(value-(dead)));
        fixedValue = Math.copySign(1, fixedValue)*Math.pow(Math.abs(fixedValue), exp);
        return MathHelp.clamp(fixedValue, -1, 1);
    }

    public void setDriveSpeed(double drivespeed){
        this.drivespeed = drivespeed;
    }
    public double getDriveSpeed(){
        return drivespeed;
    }

    public double getY(Hand hand){
        int y = hand == Hand.kRight ? 5 : 1;
        return -scaledPowerDeadband(getRawAxis(y));
    }
    public double getY(Hand hand, double exponent){
        int y = hand == Hand.kRight ? 5 : 1;
        return -scaledPowerDeadband(getRawAxis(y), exponent);
    }
    public double getX(Hand hand){
        int x = hand == Hand.kRight ? 4 : 0;
        return -scaledPowerDeadband(getRawAxis(x));
    }
    public double getX(Hand hand, double exponent){
        int x = hand == Hand.kRight ? 4 : 0;
        return -scaledPowerDeadband(getRawAxis(x), exponent);
    }

    /**
     * Applies deadband math and rate limiting to left Y to give 'forward' power.
     * Affected by controller drivespeed.
     * @return Percentage(-1 to 1)
     */
    public double getForward(){
        return forwardLimiter.calculate(getY(Hand.kLeft)*drivespeedLimiter.calculate(drivespeed));
    }
    /**
     * Applies deadband math and rate limiting to right X to give 'turn' power.
     * Affected by controller drivespeed.
     * @return Percentage(-1 to 1)
     */
    public double getTurn(){
        return turnLimiter.calculate(getX(Hand.kRight)*drivespeedLimiter.calculate(drivespeed));
    }
    /**
     * Applies deadband math and rate limiting to right X to give 'turn' power.
     * Affected by controller drivespeed-- specifically, this is different from
     * {@link #getTurn()} by increasing turning at low drivespeed and capping it at higher speeds,
     * making it more consistent.
     * @return Percentage(-1 to 1)
     */
    public double getScaledTurn(){
        double drivespeed = drivespeedLimiter.calculate(this.drivespeed);
        double adjustedDriveSpeed = Math.copySign(Math.pow(Math.abs(drivespeed), 0.2)*0.4, drivespeed);
        return turnLimiter.calculate(getX(Hand.kRight)*adjustedDriveSpeed);
    }

    public double[] getArcadeDrive(){
        double forward = getForward();
        double turn = getScaledTurn();

        return new double[]{forward-turn,forward+turn};
    }

    /**
     * Automatic left DT percentage of {@link #getArcadeDrive()}
     */
    public double getLeftArcade(){
        return getArcadeDrive()[0];
    }
    /**
     * Automatic left DT percentage of {@link #getArcadeDrive()}
     */
    public double getRightArcade(){
        return getArcadeDrive()[1];
    }
    /**
     * Simulates car steering. The ratio between left and right wheel speeds will always be the same for a given turn value,
     * regardless of the forward value-- which creates constant curvature. This also means the robot does not turn
     * when not moving forward, so {@link #kArcadeThreshold} is used to utilize arcade drive at low values.
     */
    public double[] getCurvatureDrive(){
        double forward = getForward();
        double turn = curveLimiter.calculate(getX(Hand.kRight, 1.5)*0.7);
        double left = 0;
        double right = 0;
        double forwardMagnitude = Math.abs(forward);

        if(forwardMagnitude<kArcadeThreshold){
            return new double[]{getLeftArcade(),getRightArcade()};
        }
        else{
            left = forward - forwardMagnitude*turn;
            right = forward + forwardMagnitude*turn;// positive turn increases right side
        }
        double highestSide = Math.max(left, right);
        if(highestSide>1){
            left /= highestSide;
            right /= highestSide;
        }

        return new double[]{left,right};
    }
    /**
     * Automatic left DT percentage of {@link #getCurvatureDrive(double, double, double)}
     */
    public double getLeftCurvatureDrive(){
        return getCurvatureDrive()[0];
    }
    /**
     * Automatic right DT percentage of {@link #getCurvatureDrive(double, double, double)}
     */
    public double getRightCurvatureDrive(){
        return getCurvatureDrive()[1];
    }

    /**
     * Reset the slew-rate limiters on the joysticks
     */
    public void resetLimiters(){
        forwardLimiter.reset(0);
        turnLimiter.reset(0);
        drivespeedLimiter.reset(0);
    }
}
