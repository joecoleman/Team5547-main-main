// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForwardCmd extends Command {
  private final DriveSubsystem driveSubsystem;
  private final double distance;
  private double encoderSetpoint;
  /** Creates a new DriveForwardCmd. */
  public DriveForwardCmd(DriveSubsystem driveSubsystem, double distance) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderSetpoint = driveSubsystem.getEncoderMeters() + distance;}
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setMotors(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (driveSubsystem.getEncoderMeters() >= encoderSetpoint) 
        return true;
      else 
      return false;

  }
}
