/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.common.Constants.*;

import java.util.TreeMap;

import frc.robot.common.OCConfig;
import frc.robot.states.ShooterState;
import frc.robot.common.Testable;
import frc.robot.common.OCConfig.ConfigType;
import frc.robot.util.MathHelp;

public class Shooter extends SubsystemBase implements Testable{

    private CANSparkMax shootLeft = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax shootRight = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax wrist = new CANSparkMax(7, MotorType.kBrushless);

    private double wristVolts = 0;
    private double wristTarget = ShooterWristConstants.kClearIntakeDegrees;

    private double shooterTarget = 0;
    private double lastShooterVel = 0;
    private double lastShotTime = Timer.getFPGATimestamp();

    private final double rpmTolerance = 80;
    
    private CANEncoder leftEncoder = shootLeft.getEncoder();
    private CANEncoder rightEncoder = shootRight.getEncoder();
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
    
    private SimpleMotorFeedforward leftShootFF = new SimpleMotorFeedforward(ShooterConstants.klStaticFF, ShooterConstants.klVelocityFF, 0);
    private SimpleMotorFeedforward rightShootFF = new SimpleMotorFeedforward(ShooterConstants.krStaticFF, ShooterConstants.krVelocityFF, 0);
    private SimpleMotorFeedforward wristFF = new SimpleMotorFeedforward(ShooterWristConstants.kStaticFF, ShooterWristConstants.kVelocityFF, ShooterWristConstants.kAccelerationFF);
    
    private CANPIDController leftController;
    private CANPIDController rightController;
    private ProfiledPIDController wristController =
    new ProfiledPIDController(ShooterWristConstants.kP, ShooterWristConstants.kI, ShooterWristConstants.kD,
    new Constraints(ShooterWristConstants.kVelocityConstraint, ShooterWristConstants.kAccelerationConstraint), kRobotDelta); // Positional PID controller
    
    public Shooter() {
        OCConfig.configMotors(ConfigType.SHOOTER, shootLeft, shootRight);
        OCConfig.configMotors(ConfigType.SHOOTERWRIST, wrist);

        Timer.delay(0.3);

        leftController = shootLeft.getPIDController();
        rightController = shootRight.getPIDController();

        leftController.setP(ShooterConstants.kP);
        leftController.setI(ShooterConstants.kI);
        leftController.setD(ShooterConstants.kD);
        rightController.setP(ShooterConstants.kP);
        rightController.setI(ShooterConstants.kI);
        rightController.setD(ShooterConstants.kD);

        shootLeft.burnFlash();
        shootRight.burnFlash();

        Timer.delay(0.3);
        
        shootLeft.setInverted(false);
        shootRight.setInverted(true);
        wrist.setInverted(false);

        wristController.setTolerance(0.25, 1);
    }
    
    public void periodic() {
        setWristVolts(calculateWristVolts(wristTarget));

        calculateShooterVolts(shooterTarget);
        if(shootLeft.getOutputCurrent() >= 40) lastShotTime = Timer.getFPGATimestamp();
    }

    public ShooterState getCurrentState(){
        return new ShooterState(getWristDegrees(),getRPM());
    }
    public ShooterState getTargetState(){
        return new ShooterState(wristTarget, shooterTarget);
    }

    public ProfiledPIDController getWristController(){
        return wristController;
    }
    public double getWristDegrees(){
        return (wristEncoder.get()+ShooterWristConstants.kEncoderOffset)*360;
    }
    /**
     * Calculates volts from wrist PID + FF
     * @param target Target wrist degrees
     */
    private double calculateWristVolts(double target){
        target = MathHelp.clamp(target, ShooterWristConstants.kLowestSafeDegrees, ShooterWristConstants.kHighestSafeDegrees);
        SmartDashboard.putNumber("Wrist Goal", target);
        double volts = wristController.calculate(getWristDegrees(), target);
        volts += wristFF.calculate(wristController.getGoal().velocity);
        SmartDashboard.putNumber("Wrist PID", volts);
        return volts;
    }

    public double getLeftRPM(){
        return leftEncoder.getVelocity();
    }
    public double getRightRPM(){
        return rightEncoder.getVelocity();
    }
    public double getRPM(){
        return (getLeftRPM()+getRightRPM())/2.0;
    }
    public double getTargetRPM(){
        return shooterTarget;
    }
    public double getShooterError(){
        double leftError = Math.abs(shooterTarget - getLeftRPM());
        double rightError = Math.abs(shooterTarget - getRightRPM());
        return Math.max(leftError, rightError);
    }
    public double getLastShotTime(){
        return lastShotTime;
    }
    public boolean checkIfStable(){
        double rpm = getRPM();
        boolean stable = (getShooterError() < rpmTolerance) && (Math.abs(getRPM()-lastShooterVel) < 100);
        SmartDashboard.putNumber("Shooter Error", getShooterError());
        SmartDashboard.putNumber("Shooter Accel", getRPM()-lastShooterVel);
        SmartDashboard.putBoolean("shooter Accel Stable", Math.abs(getRPM()-lastShooterVel) < 100);
        lastShooterVel = rpm;
        SmartDashboard.putBoolean("Shooter Stable", stable);
        return stable;
    }
    
    /**
     * Set wrist voltage with safety for collisions and counter-gravity
     * @param volts
     */
    public void setWristVolts(double volts){
        double minVolts = -12;
        double maxVolts = 12;
        final double deg = getWristDegrees();
        final double low = ShooterWristConstants.kLowestSafeDegrees;
        final double high = ShooterWristConstants.kHighestSafeDegrees;
        final double buffer = ShooterWristConstants.kBufferDegrees;
        if(deg<=low+buffer) minVolts = 0;
        if(deg>=high-buffer) maxVolts = 0;
        volts = MathHelp.clamp(volts, minVolts, maxVolts);
        volts += ShooterWristConstants.kCounterGravityFF*Math.cos(Units.degreesToRadians(getWristDegrees()));
        SmartDashboard.putNumber("Wrist Volts", volts);
        wrist.setVoltage(volts);
    }
    
    public void setWristPosition(double degrees){
        wristTarget = degrees;
    }
    /**
     * Calculates volts from shooter PID + FF and sets it
     * @param target Target shooter RPM
     */
    private void calculateShooterVolts(double target){
        double rps = target / 60.0;
        double leftVolts = leftShootFF.calculate(rps);
        double rightVolts = rightShootFF.calculate(rps);
        SmartDashboard.putNumber("Shooter FF", (leftVolts+rightVolts)/2);
        if(target!=0){
            leftController.setReference(target, ControlType.kVelocity, 0, leftVolts, ArbFFUnits.kVoltage);
            rightController.setReference(target, ControlType.kVelocity, 0, rightVolts, ArbFFUnits.kVoltage);
        }
        else{
            setShooterVolts(0);
        }
    }
    public void setShooterVolts(double volts){
        shootLeft.setVoltage(volts);
        shootRight.setVoltage(volts);
    }
    public void setShooterVelocity(double rpm){
        shooterTarget = rpm;
    }
    public void setState(ShooterState state){
        setShooterVelocity(state.rpm);
        setWristPosition(state.angle);
    }

    public void reset(){
        wristController.reset(getWristDegrees());
    }
    
    public void setShooterBrakeOn(boolean is){
        OCConfig.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast, shootLeft, shootRight);
    }
    public void setWristBrakeOn(boolean is){
        wrist.setIdleMode(is ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public void log(){
        SmartDashboard.putNumber("Wrist Degrees", getWristDegrees());
        SmartDashboard.putNumber("Wrist Setpoint", wristController.getSetpoint().position);
        
        SmartDashboard.putNumber("Shooter RPM", getRPM());
        SmartDashboard.putNumber("Shooter Target RPM", getTargetRPM());
        SmartDashboard.putNumber("Shooter Diff", leftEncoder.getVelocity()-rightEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Left Amps", shootLeft.getOutputCurrent());
        SmartDashboard.putNumberArray("Shooter Reference", new double[]{getRPM(), getTargetRPM()});
    }
    
    @Override
    public TestableResult test(){
        return new TestableResult("Shooter", Status.PASSED);
    }
}
