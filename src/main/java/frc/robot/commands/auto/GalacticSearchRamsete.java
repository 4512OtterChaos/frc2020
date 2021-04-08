// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auto.OCPath;
import frc.robot.auto.Paths;
import frc.robot.auto.OCPath.Preset;
import frc.robot.common.OCPhotonCam;
import frc.robot.common.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class GalacticSearchRamsete extends CommandBase {

    private OCPhotonCam camera;
    private Paths paths;
    private Drivetrain drivetrain;
    private StandardRamseteCommand ramseteCommand;

    public GalacticSearchRamsete(Drivetrain drivetrain, Paths paths, OCPhotonCam cam) {
        this.camera = cam;
        this.paths = paths;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ramseteCommand = new StandardRamseteCommand(drivetrain, paths.getGSPath(camera.findGSType()));
        ramseteCommand.initialize();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ramseteCommand.execute();
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ramseteCommand.end(interrupted);
    }
}
