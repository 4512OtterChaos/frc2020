/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.common;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.util.MathHelp;

/**
 * Custom {@link XboxController} wrapper to add convenience features for
 * driving.
 */
public class OCXboxController extends XboxController {

    // Pre-made buttons to reduce verbosity
    public final JoystickButton aButton;
    public final JoystickButton bButton;
    public final JoystickButton xButton;
    public final JoystickButton yButton;
    public final JoystickButton bumperLeftButton;
    public final JoystickButton bumperRightButton;
    public final JoystickButton backButton;
    public final JoystickButton startButton;
    public final JoystickButton stickLeftButton;
    public final JoystickButton stickRightButton;
    public final edu.wpi.first.wpilibj2.command.button.Button leftTriggerButton; // curse disambiguation
    public final edu.wpi.first.wpilibj2.command.button.Button rightTriggerButton;
    public final POVButton povUpButton;
    public final POVButton povRightButton;
    public final POVButton povDownButton;
    public final POVButton povLeftButton;

    public enum DriveMode { // Choose our tele-op control method
        TANK, TANKVOLTS, ARCADE, ARCADEVOLTS, CURVATURE, CURVATUREVOLTS
    }

    private DriveMode mode;

    private static final double kDeadband = 0.12;
    private static final double kArcadeThreshold = 0.2;

    public static final double kSpeedDefault = 0.4;
    public static final double kSpeedFast = 0.55;
    public static final double kSpeedMax = 0.9;
    private double drivespeed = kSpeedDefault;

    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(7);
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(7);
    private SlewRateLimiter curveLimiter = new SlewRateLimiter(6);
    private SlewRateLimiter drivespeedLimiter = new SlewRateLimiter(5);

    /**
     * Constructs XboxController on DS joystick port.
     */
    public OCXboxController(int port) {
        super(port);

        aButton = new JoystickButton(this, XboxController.Button.kA.value);
        bButton = new JoystickButton(this, XboxController.Button.kB.value);
        xButton = new JoystickButton(this, XboxController.Button.kX.value);
        yButton = new JoystickButton(this, XboxController.Button.kY.value);
        bumperLeftButton = new JoystickButton(this, XboxController.Button.kBumperLeft.value);
        bumperRightButton = new JoystickButton(this, XboxController.Button.kBumperRight.value);
        backButton = new JoystickButton(this, XboxController.Button.kBack.value);
        startButton = new JoystickButton(this, XboxController.Button.kStart.value);
        stickLeftButton = new JoystickButton(this, XboxController.Button.kStickLeft.value);
        stickRightButton = new JoystickButton(this, XboxController.Button.kStickRight.value);
        leftTriggerButton = new edu.wpi.first.wpilibj2.command.button.Button(() -> getTriggerAxis(Hand.kLeft) > 0.15);
        rightTriggerButton = new edu.wpi.first.wpilibj2.command.button.Button(() -> getTriggerAxis(Hand.kRight) > 0.15);
        povUpButton = new POVButton(this, 0);
        povRightButton = new POVButton(this, 90);
        povDownButton = new POVButton(this, 180);
        povLeftButton = new POVButton(this, 270);
    }

    public DriveMode getMode() {
        return mode;
    }

    public void setDriveMode(DriveMode mode) {
        this.mode = mode;
    }

    public static double deadband(double value) {
        return Math.abs(value) > kDeadband ? value : 0;
    }

    /**
     * Deadbands a value, re-scales it, and applies a power.
     * 
     * @param value Value to adjust
     * @return -1 to 1
     */
    public static double scaledPowerDeadband(double value) {
        return scaledPowerDeadband(value, 1);
    }

    /**
     * Deadbands a value, re-scales it, and applies a power.
     * 
     * @param value Value to adjust
     * @param dead  Deadband threshold
     * @return -1 to 1
     */
    public static double scaledPowerDeadband(double value, double exp) {
        final double dead = kDeadband;
        if (Math.abs(value) < dead)
            return 0;
        double scale = 1.0 / (1 - dead);
        double sign = Math.copySign(1.0, value);
        value = Math.abs(value);
        double fixedValue = sign * (scale * (value - (dead)));
        fixedValue = Math.copySign(1, fixedValue) * Math.pow(Math.abs(fixedValue), exp);
        return MathHelp.clamp(fixedValue, -1, 1);
    }

    public void setDriveSpeed(double drivespeed) {
        this.drivespeed = drivespeed;
    }

    public double getDriveSpeed() {
        return drivespeed;
    }

    public double getY(Hand hand) {
        int y = hand == Hand.kRight ? 5 : 1;
        return -scaledPowerDeadband(getRawAxis(y));
    }

    public double getY(Hand hand, double exponent) {
        int y = hand == Hand.kRight ? 5 : 1;
        return -scaledPowerDeadband(getRawAxis(y), exponent);
    }

    public double getX(Hand hand) {
        int x = hand == Hand.kRight ? 4 : 0;
        return -scaledPowerDeadband(getRawAxis(x));
    }

    public double getX(Hand hand, double exponent) {
        int x = hand == Hand.kRight ? 4 : 0;
        return -scaledPowerDeadband(getRawAxis(x), exponent);
    }

    /**
     * Applies deadband math and rate limiting to left Y to give 'forward' power.
     * Affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getForward() {
        return forwardLimiter.calculate(getY(Hand.kLeft) * drivespeedLimiter.calculate(drivespeed));
    }

    /**
     * Applies deadband math and rate limiting to right X to give 'turn' power.
     * Affected by controller drivespeed.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getTurn() {
        return turnLimiter.calculate(getX(Hand.kRight) * drivespeedLimiter.calculate(drivespeed));
    }

    /**
     * Applies deadband math and rate limiting to right X to give 'turn' power.
     * Affected by controller drivespeed-- specifically, this is different from
     * {@link #getTurn()} by increasing turning at low drivespeed and capping it at
     * higher speeds, making it more consistent.
     * 
     * @return Percentage(-1 to 1)
     */
    public double getScaledTurn() {
        double drivespeed = drivespeedLimiter.calculate(this.drivespeed);
        double adjustedDriveSpeed = Math.copySign(Math.pow(Math.abs(drivespeed), 0.2) * 0.6, drivespeed);
        return turnLimiter.calculate(getX(Hand.kRight) * adjustedDriveSpeed);
    }

    public double[] getArcadeDrive() {
        double forward = getForward();
        double turn = getScaledTurn() * 0.8;

        return new double[] { forward - turn, forward + turn };
    }

    /**
     * Automatic left DT percentage of {@link #getArcadeDrive()}
     */
    public double getLeftArcade() {
        return getArcadeDrive()[0];
    }

    /**
     * Automatic left DT percentage of {@link #getArcadeDrive()}
     */
    public double getRightArcade() {
        return getArcadeDrive()[1];
    }

    /**
     * Simulates car steering. The ratio between left and right wheel speeds will
     * always be the same for a given turn value, regardless of the forward value--
     * which creates constant curvature. This also means the robot does not turn
     * when not moving forward, so {@link #kArcadeThreshold} is used to utilize
     * arcade drive at low values.
     */
    public double[] getCurvatureDrive() {
        double forward = getForward();
        double turn = curveLimiter.calculate(getX(Hand.kRight, 1) * 0.7);
        double left = 0;
        double right = 0;
        double forwardMagnitude = Math.abs(forward);

        if (forwardMagnitude < kArcadeThreshold) {
            return new double[] { forward - turn * 0.6, forward + turn * 0.6 };
        } else {
            left = forward - forwardMagnitude * turn;
            right = forward + forwardMagnitude * turn;// positive turn increases right side
        }
        double highestSide = Math.max(left, right);
        if (highestSide > 1) {
            left /= highestSide;
            right /= highestSide;
        }

        return new double[] { left, right };
    }

    /**
     * Automatic left DT percentage of
     * {@link #getCurvatureDrive(double, double, double)}
     */
    public double getLeftCurvatureDrive() {
        return getCurvatureDrive()[0];
    }

    /**
     * Automatic right DT percentage of
     * {@link #getCurvatureDrive(double, double, double)}
     */
    public double getRightCurvatureDrive() {
        return getCurvatureDrive()[1];
    }

    /**
     * Reset the slew-rate limiters on the joysticks
     */
    public void resetLimiters() {
        forwardLimiter.reset(0);
        turnLimiter.reset(0);
        drivespeedLimiter.reset(0);
    }
}
