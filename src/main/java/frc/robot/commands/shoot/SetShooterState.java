/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.SAS;

public class SetShooterState extends CommandBase {
    
    private final Shooter shooter;
    private ShooterState state;
    private SAS analysis;
    private Limelight limelight;
    private boolean current = false; //using current wrist position
    private boolean started = false;
    
    public SetShooterState(Shooter shooter, ShooterState state) {
        this.shooter = shooter;
        this.state = state;
        
        addRequirements(shooter);
    }
    public SetShooterState(Shooter shooter, SAS analysis, Limelight limelight) {
        this.shooter = shooter;
        this.analysis = analysis;
        this.limelight = limelight;
        this.state = analysis.findShot(limelight.getTrigDistance());
        
        addRequirements(shooter);
    }
    public SetShooterState(Shooter shooter) {
        this.shooter = shooter;
        this.state = new ShooterState(shooter);
        this.current = true;
        
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        started = true;
        if(current) state = new ShooterState(shooter);
        shooter.getWristController().reset(shooter.getWristDegrees());
        shooter.setState(state);
    }
    
    @Override
    public void execute() {
        if(limelight != null){
            shooter.setState(analysis.findShot(limelight.getTrigDistance()));
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        started = false;
    }
    
    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("Wrist Error", shooter.getWristController().getPositionError());
        SmartDashboard.putNumber("Shooter Error", shooter.getShooterError());
        return started && shooter.getWristController().atGoal() && shooter.checkIfStable();
    }
}
