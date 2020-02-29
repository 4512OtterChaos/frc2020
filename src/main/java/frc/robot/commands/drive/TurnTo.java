/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

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
    
    private static final double kCruiseVelocityDegrees = Units.radiansToDegrees(kMaxVelocityRadians)*0.8;
    
    private static ProfiledPIDController controller = new ProfiledPIDController(0.04, 0, 0, 
    new TrapezoidProfile.Constraints(kCruiseVelocityDegrees, kCruiseVelocityDegrees*2),
    Constants.kRobotDelta);
    private static boolean resetFlag = false;
    private boolean started = false;
    
    public TurnTo(Drivetrain drivetrain, double target) {
        super(
            controller,
            () -> drivetrain.getYawPosition(),
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
            () -> drivetrain.getYawPosition(),
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
        started = true;
        controller.setTolerance(0.4, 3);
    }
    
    @Override
    public void execute(){
        super.execute();
        SmartDashboard.putNumber("TurnTo Target", controller.getGoal().position);
    }
    
    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        started = false;
    }
    
    
    @Override
    public boolean isFinished() {
        return getController().atGoal() && started;
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
        }
        );
        
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
        }
        );
        return turnToLimelightTarget;
    }
    
    public static Command createSimplerTurnToTarget(Drivetrain drivetrain, Limelight limelight){
        TurnTo turnToLimelightTarget = new TurnTo(drivetrain,
        ()->{
            double heading = drivetrain.getYawPosition()-limelight.getTx();
            //FieldUtil.getRelativePose(Rotation2d.fromDegrees(heading), Units.inchesToMeters(limelight.getTrigDistance()));
            return heading;
        }
        );
        return turnToLimelightTarget;
    }
}
