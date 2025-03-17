// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristUpPIDCmd extends Command {
  private final WristSubsystem wristSubsystem;
  private final PIDController pidController;
  
  /** Creates a new ElevatorUpPIDCmd. */

  public WristUpPIDCmd(WristSubsystem wristSubsystem, double setpoint) {
    this.wristSubsystem = wristSubsystem;
    this.pidController = new PIDController(0.1, 0.1, 0.1);
    pidController.setSetpoint(setpoint);
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(wristSubsystem.getEncoderDistance());
    wristSubsystem.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
