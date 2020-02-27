/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FieldUtil;

public class TurnHomingOuter extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final double heading;

    public TurnHomingOuter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.heading = FieldUtil.getRelativeHeading(drivetrain.getOdometry().getPoseMeters(), VisionConstants.kTargetTranslation).getDegrees();
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
