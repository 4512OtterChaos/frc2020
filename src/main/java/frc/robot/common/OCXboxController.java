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

    private static final double deadband = 0.12;

    private double drivespeed = 1;
    public static final double kSpeedDefault = 0.35;
    public static final double kSpeedFast = 0.55;
    public static final double kSpeedMax = 0.8;
    
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(3.75);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(4);
    private SlewRateLimiter drivespeedLimiter = new SlewRateLimiter(4);

    /**
     * Constructs XboxController on DS joystick port.
     */
    public OCXboxController(int port){
        super(port);
    }

    /**
     * Deadbands a value and re-scales it.
     * @param value Value to adjust
     * @return -1 to 1
     */
    public static double scaledDeadband(double value){
        return scaledDeadband(value, deadband);
    }
    /**
     * Deadbands a value and re-scales it.
     * @param value Value to adjust
     * @param dead Deadband threshold
     * @return -1 to 1
     */
    public static double scaledDeadband(double value, double dead){
        if(Math.abs(value) < dead) return 0;
        double scale = 1.0 / (1 - dead);
        double sign = Math.copySign(1.0, value);
        value = Math.abs(value);
        double fixedValue = sign*(scale*((value-(sign*dead))*value));
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
        return -scaledDeadband(getRawAxis(y));
    }
    public double getX(Hand hand){
        int x = hand == Hand.kRight ? 4 : 0;
        return -scaledDeadband(getRawAxis(x));
    }

    /**
     * Applies deadband math and rate limiting to left Y to give 'forward' power.
     * Affected by controller drivespeed.
     * @return Percentage(-1 to 1)
     */
    public double getForward(){
        return forwardLimiter.calculate(getY(Hand.kLeft))*drivespeedLimiter.calculate(drivespeed);
    }
    /**
     * Applies deadband math and rate limiting to right X to give 'turn' power.
     * Affected by controller drivespeed.
     * @return Percentage(-1 to 1)
     */
    public double getTurn(){
        return turnLimiter.calculate(getX(Hand.kRight))*drivespeedLimiter.calculate(drivespeed);
    }

    /**
     * Gives left DT percentage using arcade mode.
     */
    public double getLeftArcade(){
        return getLeftArcade(getForward(), getTurn());
    }
    /**
     * Gives simple left DT percentage using arcade mode.
     * WARNING: Does not use rate limiters
     */
    public double getLeftArcade(double forward, double turn){
        return forward - turn;
    }
    /**
     * Gives right DT percentage using arcade mode.
     */
    public double getRightArcade(){
        return getRightArcade(getForward(), getTurn());
    }
    /**
     * Gives simple right DT percentage using arcade mode.
     * WARNING: Does not use rate limiters
     */
    public double getRightArcade(double forward, double turn){
        return forward + turn;
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
