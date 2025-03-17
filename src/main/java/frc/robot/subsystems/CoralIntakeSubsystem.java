package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;

    public CoralIntakeSubsystem() {
        intakeMotor = new SparkMax(Constants.CoralIntakeConstants.intakeMotor, MotorType.kBrushless); // CAN ID 15
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void intake(double speed) {
        intakeMotor.set(speed); // Set motor to intake
    }

    public void eject() {
        intakeMotor.set(-.5); // Set motor to eject
    }

    public void stop() {
        intakeMotor.stopMotor(); // Stop the motor
    }

    public void setMotor(double speed) {
        
    }public double getEncoderDistance() {

        // Replace with actual implementation to get encoder distance
    
        return 0.5;}
}