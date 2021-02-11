/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.common.Constants;
import frc.robot.common.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.FieldUtil;

import static frc.robot.common.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

public class TurnTo extends ProfiledPIDCommand {
    
    private final Drivetrain drivetrain;
    
    private static final double kCruiseVelocityDegrees = Units.radiansToDegrees(kMaxVelocityRadians)*0.4;

    private static final double kVelocityToleranceDegrees = 5;
    
    private static ProfiledPIDController controller = new ProfiledPIDController(0.016, 0, 0, 
    new TrapezoidProfile.Constraints(kCruiseVelocityDegrees, kCruiseVelocityDegrees*5),
    Constants.kRobotDelta);
    private boolean started = false;
    
    public TurnTo(Drivetrain drivetrain, double target) {
        super(
            controller,
            () -> drivetrain.getContinuousYawPosition(),
            () -> target,
            (output, setpoint) -> {
                drivetrain.setChassisSpeed(0, output);
            }
        );
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    public TurnTo(Drivetrain drivetrain, DoubleSupplier target) {
        super(
            controller,
            () -> drivetrain.getContinuousYawPosition(),
            target,
            (output, setpoint) -> {
                drivetrain.setChassisSpeed(0, output);
            }
        );
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    
    /*
    @Override
    public void initialize(){
        if(resetFlag){
            controller.reset(drivetrain.getYawPosition(), drivetrain.getYawVelocity());
            resetFlag = false;
        }
    }
    */
    @Override
    public void initialize(){
        super.initialize();
        controller.enableContinuousInput(-180, 180);
        controller.setTolerance(0.4, 6);
    }
    
    @Override
    public void execute(){
        super.execute();
        drivetrain.setTurnToTarget(controller.getGoal().position);
        SmartDashboard.putNumber("TurnTo Target", controller.getGoal().position);
        SmartDashboard.putNumber("TurnTo State Target", controller.getSetpoint().position);
    }
    
    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        started = false;
    }

    public boolean atGoal(){
        SmartDashboard.putNumber("TurnTo Error", controller.getPositionError());
        
        boolean atGoal = getController().atGoal();
        double velocity = drivetrain.getYawVelocity();
        boolean stable = velocity <= kVelocityToleranceDegrees;

        return atGoal && stable;
    }
    
    @Override
    public boolean isFinished() {
        return atGoal() && started;
    }
    
    /**
    * Returns a TurnTo command that will turn to the target translation given robot pose.
    * It 'homes' as there is one constant heading that points to the target, which is not updated.
    */
    public static TurnTo createHomeToTarget(Drivetrain drivetrain, Translation2d targetTranslation){
        return new TurnTo(drivetrain, FieldUtil.getTargetedHeading(drivetrain.getOdometry().getPoseMeters(), targetTranslation).getDegrees());
    }
    
    public static Command createTurnToTarget(Drivetrain drivetrain, Limelight limelight){
        TurnTo turnToLimelightTarget = new TurnTo(drivetrain,
        ()->{
            double heading = drivetrain.getPoseFromHistory(limelight.getLatencySeconds()).getRotation().getDegrees()-limelight.getTx();
            //FieldUtil.getRelativePose(Rotation2d.fromDegrees(heading), Units.inchesToMeters(limelight.getTrigDistance()));
            return heading;
        });
        
        return new ConditionalCommand(
        turnToLimelightTarget,
        createHomeToTarget(drivetrain, VisionConstants.kTargetTranslation)
        .withInterrupt(limelight::getHasTarget)
        .andThen(createTurnToTarget(drivetrain, limelight)), 
        limelight::getHasTarget
        );
    }
    
    public static Command createSimpleTurnToTarget(Drivetrain drivetrain, Limelight limelight){
        TurnTo turnToLimelightTarget = new TurnTo(drivetrain,
        ()->{
            double heading = drivetrain.getPoseFromHistory(limelight.getLatencySeconds()).getRotation().getDegrees()-limelight.getTx();
            //FieldUtil.getRelativePose(Rotation2d.fromDegrees(heading), Units.inchesToMeters(limelight.getTrigDistance()));
            return heading;
        });
        return turnToLimelightTarget.withTimeout(1.4);
    }
    
    public static Command createSimplerTurnToTarget(Drivetrain drivetrain, Limelight limelight){
        TurnTo turnToLimelightTarget = new TurnTo(drivetrain,
        ()->{
            double heading = drivetrain.getContinuousYawPosition()-limelight.getTx();
            //FieldUtil.getRelativePose(Rotation2d.fromDegrees(heading), Units.inchesToMeters(limelight.getTrigDistance()));
            return heading;
        });
        return turnToLimelightTarget.withTimeout(1.4);
    }
}
