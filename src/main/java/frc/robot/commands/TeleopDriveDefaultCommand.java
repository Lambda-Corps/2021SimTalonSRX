// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;

/** An example command that uses an example subsystem. */
public class TeleopDriveDefaultCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;
  private final XboxController m_driverController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDriveDefaultCommand(DriveTrain subsystem, XboxController driverController) {
    m_driveTrain = subsystem;
    m_driverController = driverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Axis are inverted, negate them so positive is forward
    double turn = m_driverController.getRawAxis(DRIVER_RIGHT_AXIS); // Right X
    double move = -m_driverController.getRawAxis(DRIVER_LEFT_AXIS); // Left Y

    SmartDashboard.putNumber("Move", move);
    SmartDashboard.putNumber("Turn", turn);
    m_driveTrain.manualDrive(move, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
