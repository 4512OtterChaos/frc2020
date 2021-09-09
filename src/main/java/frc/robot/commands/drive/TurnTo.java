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
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.common.Constants;
import frc.robot.common.OCPhotonCam;
import frc.robot.common.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldUtil;

import static frc.robot.common.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

public class TurnTo extends ProfiledPIDCommand {
    
    private final Drivetrain drivetrain;
    
    private static final double kCruiseVelocityDegrees = Units.radiansToDegrees(kMaxVelocityRadians*0.3);
    
    private static final double kPositionToleranceDegrees = 0.8;
    private static final double kVelocityToleranceDegrees = 4;
    
    private static ProfiledPIDController controller = new ProfiledPIDController(0.45, 0.075, 0, 
        new TrapezoidProfile.Constraints(kCruiseVelocityDegrees, kCruiseVelocityDegrees*1.75),
        Constants.kRobotDelta
    );
    
    public TurnTo(Drivetrain drivetrain, double target) {
        super(
        controller,
        () -> drivetrain.getHeading().getDegrees(),
        () -> target,
        (output, setpoint) -> {
            drivetrain.setChassisSpeed(new ChassisSpeeds(0,0,Units.degreesToRadians(setpoint.velocity + output)));
        },
        drivetrain
        );
        this.drivetrain = drivetrain;
    }
    public TurnTo(Drivetrain drivetrain, DoubleSupplier target) {
        super(
        controller,
        () -> drivetrain.getHeading().getDegrees(),
        target,
        (output, setpoint) -> {
            drivetrain.setChassisSpeed(new ChassisSpeeds(0,0,Units.degreesToRadians(setpoint.velocity + output)));
        },
        drivetrain
        );
        this.drivetrain = drivetrain;
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
        controller.setTolerance(kPositionToleranceDegrees, kVelocityToleranceDegrees);
    }
    
    @Override
    public void execute(){
        super.execute();
        drivetrain.setTurnToTarget(controller.getGoal().position);
        drivetrain.setTurnToError(this.getController().getPositionError());
        SmartDashboard.putNumber("TurnTo Target", controller.getGoal().position);
        SmartDashboard.putNumber("TurnTo State Target", controller.getSetpoint().position);
    }
    
    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
    }
    
    public boolean atGoal(){
        SmartDashboard.putNumber("TurnTo Error", controller.getPositionError());
        
        boolean atGoal = getController().atGoal();
        double velocity = drivetrain.getYawVelocity();
        
        return atGoal;
    }
    
    @Override
    public boolean isFinished() {
        return atGoal();
    }
    
    /**
    * Returns a TurnTo command that will turn to the target translation given robot pose.
    * It 'homes' to a static field position since the robot estimates its own position.
    */
    public static TurnTo createHomeToTarget(Drivetrain drivetrain, Translation2d targetTranslation){
        double relativeAngle = FieldUtil.getRelativeAngle(drivetrain.getOdometry().getPoseMeters().getTranslation(), targetTranslation).getDegrees();
        return new TurnTo(drivetrain, drivetrain.getHeading().plus(Rotation2d.fromDegrees(relativeAngle)).getDegrees());
    }
    
    /**
     * Returns a TurnTo command that will turn to the target using tracked targets from a camera.
     * Tracked target positions are applied to previous robot positions to account for latency.
     * When targets are not visible, the field position of the target will be used instead.
     */
    public static Command createTurnToTarget(Drivetrain drivetrain, OCPhotonCam camera){
        TurnTo turnToCameraTarget = new TurnTo(drivetrain,
        ()->{
            PhotonPipelineResult visionResult = camera.getLatestResult();
            PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
            if(!visionResult.hasTargets()) return drivetrain.getHeading().getDegrees();
            double heading = drivetrain.getPoseFromHistory(visionResult.getLatencyMillis()/1000.0).getRotation().plus(Rotation2d.fromDegrees(visionTarget.getYaw())).getDegrees();
            //FieldUtil.getRelativePose(Rotation2d.fromDegrees(heading), Units.inchesToMeters(camera.getTrigDistance()));
            return heading;
        });
        
        return new ConditionalCommand(
        turnToCameraTarget,
        createHomeToTarget(drivetrain, VisionConstants.kTargetTranslation)
        .withInterrupt(camera::hasTargets)
        .andThen(createTurnToTarget(drivetrain, camera)), 
        camera::hasTargets
        ).withTimeout(2);
    }
    
    /**
     * Returns a TurnTo command that will turn to the target using tracked targets from a camera.
     * Tracked target positions are applied to previous robot positions to account for latency.
     */
    public static Command createSimpleTurnToTarget(Drivetrain drivetrain, OCPhotonCam camera){
        TurnTo turnToCameraTarget = new TurnTo(drivetrain,
        ()->{
            PhotonPipelineResult visionResult = camera.getLatestResult();
            PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
            if(!visionResult.hasTargets()) return drivetrain.getHeading().getDegrees();
            double heading = drivetrain.getPoseFromHistory(visionResult.getLatencyMillis()/1000.0).getRotation().plus(Rotation2d.fromDegrees(visionTarget.getYaw())).getDegrees();
            //FieldUtil.getRelativePose(Rotation2d.fromDegrees(heading), Units.inchesToMeters(camera.getTrigDistance()));
            return heading;
        });
        return turnToCameraTarget.withTimeout(1.4);
    }
    /**
     * Returns a TurnTo command that will turn to the target using tracked targets from a camera.
     */
    public static Command createSimplerTurnToTarget(Drivetrain drivetrain, OCPhotonCam camera){
        TurnTo turnToCameraTarget = new TurnTo(drivetrain,
        ()->{
            PhotonPipelineResult visionResult = camera.getLatestResult();
            PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
            if(!visionResult.hasTargets()) return drivetrain.getHeading().getDegrees();
            double heading = drivetrain.getHeading().plus(Rotation2d.fromDegrees(visionTarget.getYaw())).getDegrees();
            //FieldUtil.getRelativePose(Rotation2d.fromDegrees(heading), Units.inchesToMeters(camera.getTrigDistance()));
            return heading;
        });
        return turnToCameraTarget.withTimeout(1.4);
    }
    
    /**
    * Turns by always moving one side forward. This keeps the chain in tension and avoids backlash.
    */
    public static Command createTensionedTurnToTarget(Drivetrain drivetrain, OCPhotonCam camera){
        ProfiledPIDCommand tensionedTurn = new ProfiledPIDCommand(
        new ProfiledPIDController(0.25, 0, 0, new TrapezoidProfile.Constraints(kCruiseVelocityDegrees, kCruiseVelocityDegrees*1.5), Constants.kRobotDelta),
        () -> drivetrain.getHeading().getDegrees(),
        () -> {
            PhotonPipelineResult visionResult = camera.getLatestResult();
            PhotonTrackedTarget visionTarget = visionResult.getBestTarget();
            if(!visionResult.hasTargets()) return drivetrain.getHeading().getDegrees();
            return drivetrain.getHeading().plus(Rotation2d.fromDegrees(visionTarget.getYaw())).getDegrees();
        },
        (output, setpoint) -> {
            double tensionVolts = drivetrain.getLinearFF().ks * 0.4; // slight voltage for putting in tension
            double leftVolts = tensionVolts;
            double rightVolts = tensionVolts;
            
            if(output > 0) rightVolts += output;
            else if(output < 0) leftVolts -= output;
            
            drivetrain.tankDriveVolts(leftVolts, rightVolts);
        },
        drivetrain
        ){
            @Override
            public void execute() {
                super.execute();
                double goalPos = this.getController().getGoal().position;
                double errorPos = this.getController().getPositionError();
                drivetrain.setTurnToTarget(goalPos);
                drivetrain.setTurnToError(errorPos);
                SmartDashboard.putNumber("TurnTo Target", goalPos);
                SmartDashboard.putNumber("TurnTo State Target", this.getController().getSetpoint().position);
                SmartDashboard.putNumber("TurnTo Error", errorPos);
            }

            @Override
            public boolean isFinished(){
                return getController().atGoal();
            }
        };
        ProfiledPIDController controller = tensionedTurn.getController();
        controller.enableContinuousInput(-180, 180);
        controller.setTolerance(kPositionToleranceDegrees, kVelocityToleranceDegrees);
        return tensionedTurn;
    }
}
