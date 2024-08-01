package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PositionArm;

public class StagingZone extends SubsystemBase {
    CANSparkMax stagingMotor;
    CANSparkMax armMotor;
    DutyCycleEncoder armAbsoluteEncoder = new DutyCycleEncoder(9);

    DigitalInput frontNoteSensor;
    DigitalInput backNoteSensor;

    public StagingZone(int stagingMotorCanId, int armMotorCanId, int frontSensorChannel, int backSensorChannel) {
        stagingMotor = new CANSparkMax(stagingMotorCanId, MotorType.kBrushless);
        armMotor = new CANSparkMax(armMotorCanId, MotorType.kBrushless);

        frontNoteSensor = new DigitalInput(frontSensorChannel);
        backNoteSensor = new DigitalInput(backSensorChannel);
    }

    public boolean getFrontSensor() {
        return !frontNoteSensor.get();
    }
    public boolean getBackSensor() {
        return !backNoteSensor.get();
    }

    public Command setStagingMotorSpeed(double speed) {
        return runOnce(
            () -> {
                stagingMotor.set(speed);
            }
        );
    }

    public void setArmMotorSpeed(double speed) {
        armMotor.set(speed);
    }

    public double getAdjustedAbsoluteEncoder() {
        double absoluteEncoderPosition = armAbsoluteEncoder.getAbsolutePosition();
        absoluteEncoderPosition += 0.5;

        if (absoluteEncoderPosition > 1.0) {
            absoluteEncoderPosition -= 1.0;
        }

        return absoluteEncoderPosition;
    }
}
