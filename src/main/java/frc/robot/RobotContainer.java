// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.BasicDriveAuton;
import frc.robot.commands.CoralEjectCmd;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.ElevatorUpPIDCmd;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.WristUpPIDCmd;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;





/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems
  private final Gyro m_gyro = new Gyro();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);
  private final AlgaeIntakeSubsystem m_algaeIntake = new AlgaeIntakeSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final CoralIntakeSubsystem m_coralIntake = new CoralIntakeSubsystem();
  private final WristSubsystem m_wrist = new WristSubsystem();
  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // The operators controller
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("DriveForward", new DriveForwardCmd(m_robotDrive, 1.0));
    NamedCommands.registerCommand("ElevatorUp", new ElevatorUpPIDCmd(m_elevator, 1.0));
    NamedCommands.registerCommand("WristUp", new WristUpPIDCmd(m_wrist, 1.0));
    NamedCommands.registerCommand("CoralEject", new CoralEjectCmd(m_coralIntake, .2));
        
    
    autoChooser = AutoBuilder.buildAutoChooser("Auto1");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.x().whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    //m_driverController.y().whileTrue(new InstantCommand(
     //       () -> m_robotDrive.resetGyro(),
     //       m_robotDrive));
    
   
    // AlgaeIntake
    m_driverController.rightBumper().whileTrue(new InstantCommand(() -> m_algaeIntake.intake(0.7)))
        .onFalse(new InstantCommand(() -> m_algaeIntake.stop()));
    m_driverController.leftBumper().whileTrue(new InstantCommand(() -> m_algaeIntake.eject()))
        .onFalse(new InstantCommand(() -> m_algaeIntake.stop()));
    // Climber
    m_operatorController.start().whileTrue(new InstantCommand(() -> m_climber.climb(1)))
        .onFalse(new InstantCommand(() -> m_climber.stopClimb()));
        m_operatorController.back().whileTrue(new InstantCommand(() -> m_climber.climb(-1)))
        .onFalse(new InstantCommand(() -> m_climber.stopClimb()));    
    // CoralIntake
    m_operatorController.leftTrigger(0.1).whileTrue(new InstantCommand(() -> m_coralIntake.intake(0.2)))
        .onFalse(new InstantCommand(() -> m_coralIntake.stop()));
    m_operatorController.leftBumper().whileTrue(new InstantCommand(() -> m_coralIntake.eject()))
        .onFalse(new InstantCommand(() -> m_coralIntake.stop()));
    // Wrist
    m_operatorController.x().whileTrue(new InstantCommand(() -> m_wrist.moveUp(1)))
        .onFalse(new InstantCommand(() -> m_wrist.hold()));
    m_operatorController.y().whileTrue(new InstantCommand(() -> m_wrist.moveDown(1)))
        .onFalse(new InstantCommand(() -> m_wrist.stop()));
    // Elevator
    m_driverController.rightTrigger().whileTrue(new InstantCommand(() -> m_elevator.move(0.5))).onFalse(new InstantCommand(() -> m_elevator.hold()));;
    m_driverController.leftTrigger().whileTrue(new InstantCommand(() -> m_elevator.move(-0.2))).onFalse(new InstantCommand(() -> m_elevator.stop()));
    // Elevator, Speed Controlled by Triggers
    // m_driverController.rightTrigger().whileTrue(new InstantCommand(() -> m_elevator.move(m_driverController.getRightTriggerAxis()))).onFalse(new InstantCommand(() -> m_elevator.hold()));;
    // m_driverController.leftTrigger().whileTrue(new InstantCommand(() -> m_elevator.move(-m_driverController.getLeftTriggerAxis()))).onFalse(new InstantCommand(() -> m_elevator.stop()));

    m_driverController.y().onTrue(new ResetGyro(m_robotDrive));
  }

  /**
   * pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
