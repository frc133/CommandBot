// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.StagingZone;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PositionArm extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final StagingZone stagingZone;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */

    PIDController armPID;
    double armTarget;

    final double ARM_P = 7.0;
    final double ARM_I = 0.06;
    final double ARM_D = 0.02;
    final double ARM_MOTOR_UP_CAP = -1.0;
    final double ARM_MOTOR_DOWN_CAP = 0.85;

    public PositionArm(StagingZone subsystem, double target) {
        stagingZone = subsystem;
        armTarget = target;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armPID = new PIDController(ARM_P, ARM_I, ARM_D);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        stagingZone.setArmMotorSpeed(MathUtil.clamp(armPID.calculate(stagingZone.getAdjustedAbsoluteEncoder(), armTarget), ARM_MOTOR_UP_CAP, ARM_MOTOR_DOWN_CAP));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        stagingZone.setArmMotorSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(stagingZone.getAdjustedAbsoluteEncoder() - armTarget) < 0.01;
    }
}
