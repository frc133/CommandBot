// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.StagingZone;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeNote extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Intake intake;
  private final StagingZone stagingZone;
  
  final double INTAKE_MOTOR_SPEED = 1.0;

  final double STAGING_MOTOR_SPEED = 1.0;
  final double STAGING_MOTOR_ALIGNMENT_SPEED = 0.4;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeNote(Intake intakeSubsystem, StagingZone stagingZoneSubsystem) {
    intake = intakeSubsystem;
    stagingZone = stagingZoneSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(stagingZone);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeSpeed(INTAKE_MOTOR_SPEED);
    stagingZone.setStagingMotorSpeed(-STAGING_MOTOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(INTAKE_MOTOR_SPEED);
    if (stagingZone.getFrontSensor()) {
      stagingZone.setStagingMotorSpeed(-STAGING_MOTOR_ALIGNMENT_SPEED);
    } else {
      stagingZone.setStagingMotorSpeed(-STAGING_MOTOR_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
    stagingZone.setStagingMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stagingZone.getFrontSensor() && stagingZone.getBackSensor();
  }
}
