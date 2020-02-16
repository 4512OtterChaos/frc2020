/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Pair;

/**
 * Class for holding commandGroups defining different auto options.
 */
public class AutoList {
    SendableChooser<Command> stage1Options = new SendableChooser<>();
    SendableChooser<Command> stage2Options = new SendableChooser<>();
    SendableChooser<Command> stage3Options = new SendableChooser<>();
    SendableChooser<Command> stage4Options = new SendableChooser<>();
    SendableChooser<Command> fullAutoOptions = new SendableChooser<>();

    private final Command nothing;

    /**
     * Constructs different auto options given drivetrain.
     */
    public AutoList(Drivetrain drivetrain){
        nothing = new InstantCommand(()->drivetrain.tankDrive(0,0), drivetrain);

        stage1Options.setDefaultOption("Nothing", nothing);
    }

    public Command getSelected(){
        Command selected;
        if(fullAutoOptions.getSelected()==nothing){
            selected = 
                stage1Options.getSelected().andThen(
                stage2Options.getSelected()).andThen(
                stage3Options.getSelected()).andThen(
                stage4Options.getSelected());
        }
        else{
            selected = fullAutoOptions.getSelected();
        }
        return selected;
    }
    public void submit(){
        SmartDashboard.putData("Stage 1 Options", stage1Options);
        SmartDashboard.putData("Stage 2 Options", stage2Options);
        SmartDashboard.putData("Stage 3 Options", stage3Options);
        SmartDashboard.putData("Stage 4 Options", stage4Options);
        SmartDashboard.putData("Full Auto Options", fullAutoOptions);
    }
}
