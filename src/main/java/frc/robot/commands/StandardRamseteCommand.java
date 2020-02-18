/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.RamseteController;
import frc.robot.auto.RamseteCommand;
import static frc.robot.common.Constants.*;
import static frc.robot.common.Constants.AutoConstants.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.auto.OCPath;
import frc.robot.auto.Paths;

/**
 * A basic example of using ramsete controller to follow a short path
 */
public class StandardRamseteCommand extends RamseteCommand{

    private final Drivetrain drivetrain;
    private final OCPath trajectory;
    private final static RamseteController controller = new RamseteController(); // static is fine as most of the functionality is in calculate()
    private Timer timer = new Timer();
    private double lastTime;
    private double timeOffReference = 0;
    private static boolean abandonedTrajectory = false; // failure flag

    /**
     * Construct ramsete command using drivetrain and following given trajectory
     */
    public StandardRamseteCommand(Drivetrain drivetrain, OCPath trajectory){
        super(
            trajectory,
            () -> drivetrain.getOdometry().getPoseMeters(),
            controller,
            drivetrain.getFeedForward(),
            drivetrain.getKinematics(),
            drivetrain::getWheelSpeeds,
            drivetrain.getLeftPIDController(),
            drivetrain.getRightPIDController(),
            drivetrain::tankDriveVolts,
            drivetrain
        );

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;

        controller.setTolerance(kAcceptablePoseError);
    }

    @Override
    public void initialize(){
        super.initialize();

        lastTime = Timer.getFPGATimestamp();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        super.execute();

        double dt = Timer.getFPGATimestamp()-lastTime;
        if(!controller.atReference()){
            timeOffReference += dt;
        }
        else{
            timeOffReference = Math.max(0, timeOffReference-3*dt);
        }
        if(timeOffReference>kReferenceFailureWindow){
            abandonedTrajectory = true;
            end(true);
        }
        lastTime = Timer.getFPGATimestamp();

        Paths.logTrajectory(trajectory, timer.get());
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);

        Paths.logTrajectory(trajectory, trajectory.getTotalTimeSeconds()+1);
        timer.stop();
        drivetrain.tankDrive(0, 0);
    }

    public RamseteController getController(){
        return controller;
    }
    public OCPath getPath(){
        return trajectory;
    }
    /**
     * Returns true if the last trajectory followed was abandoned due to being outside acceptable tolerance for too long.
     * Resets on new command initialization.
     * Since this method is static, it is useful as a global flag for interrupting autonomous command groups.
     */
    public static boolean getFailedTrajectory(){
        return abandonedTrajectory;
    }
}
