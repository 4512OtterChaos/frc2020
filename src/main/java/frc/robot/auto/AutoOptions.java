/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.util.Pair;

/**
 * Class for holding commandGroups defining different auto options.
 */
public class AutoOptions {
    // Chooser boxes for creating autos
    SendableChooser<Command> stage1Options = new SendableChooser<>();
    SendableChooser<Command> stage2Options = new SendableChooser<>();
    SendableChooser<Command> stage3Options = new SendableChooser<>();
    SendableChooser<Command> stage4Options = new SendableChooser<>();
    SendableChooser<Command> fullAutoOptions = new SendableChooser<>();

    // Maps of modular commands as well as complete preset auto command groups
    HashMap<String, Command> stageOptions = new HashMap<String, Command>();
    HashMap<String, Command> fullOptions = new HashMap<String, Command>();

    /**
     * Constructs different auto options given drivetrain.
     */
    public AutoOptions(Drivetrain drivetrain, Intake intake, Indexer indexer){
        Command nothing = new InstantCommand(()->drivetrain.tankDrive(0,0), drivetrain);

        // add modular/preset commands
        stageOptions.put("Nothing", nothing);
        fullOptions.put("Nothing", nothing);

        // populate sendable choosers with constructed commands
        putStageDefaultOption("Nothing", nothing);
        fullAutoOptions.setDefaultOption("Nothing", nothing);

        for(Entry<String,Command> entry:stageOptions.entrySet()){
            putStageOption(entry.getKey(), entry.getValue());
        }
        for(Entry<String,Command> entry:fullOptions.entrySet()){
            fullAutoOptions.addOption(entry.getKey(), entry.getValue());
        }
    }

    private void putStageOption(String name, Command command){
        stage1Options.addOption(name, command);
        stage2Options.addOption(name, command);
        stage3Options.addOption(name, command);
        stage4Options.addOption(name, command);
    }
    private void putStageDefaultOption(String name, Command command){
        stage1Options.setDefaultOption(name, command);
        stage2Options.setDefaultOption(name, command);
        stage3Options.setDefaultOption(name, command);
        stage4Options.setDefaultOption(name, command);
    }

    /**
     * Constructs a command group based off of selected stage commands.
     * If a preset auto is selected, it will use that instead.
     */
    public Command getSelected(){
        Command selected;
        if(fullAutoOptions.getSelected()==fullOptions.get("Nothing")){
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
    /**
     * Display sendable choosers
     */
    public void submit(){
        SmartDashboard.putData("Stage 1 Options", stage1Options);
        SmartDashboard.putData("Stage 2 Options", stage2Options);
        SmartDashboard.putData("Stage 3 Options", stage3Options);
        SmartDashboard.putData("Stage 4 Options", stage4Options);
        SmartDashboard.putData("Full Auto Options", fullAutoOptions);
    }
}
