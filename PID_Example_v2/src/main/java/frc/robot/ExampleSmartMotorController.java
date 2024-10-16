package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class ExampleSmartMotorController {

    public enum PIDMode { //enum creates a set of final variables -- usually declared in all caps?
        kPosition,
        kVelocity,
        kMovementWitchcraft;
    }

    private final WPI_TalonSRX m_motor;
    private double m_kp = 0.5;  // Proportional
    private double m_ki = 0.0;   // Integral
    private double m_kd = 0.0;   // Derivative
    private double error;

    private double m_setpoint = 0;
    private boolean PIDOn = false;

    public ExampleSmartMotorController(int port) {
        m_motor = new WPI_TalonSRX(port);
    }

    public double getError(double goal, double currentPosition) {
        return goal - currentPosition;
    }

    public void setPID(double kp, double ki, double kd) {
        m_motor.config_kP(0, kp);
        m_motor.config_kI(0, ki);
        m_motor.config_kD(0, kd);
    }

    public void setPIDOn(boolean pid) {
      PIDOn = pid;
    }

    public boolean returnPIDOn(){
      return PIDOn; 
    }

    public void setSetpoint(PIDMode mode, double setpoint, double arbFeedForward) {
        m_motor.set(ControlMode.Position,setpoint);
        PIDOn = true;
    }

    public void updateMotorSpeed() {
        while (PIDOn == true) {
            double currentPosition = getEncoderDistance();
            error = m_setpoint - currentPosition;

            // Calculate output
            double output = (m_kp * error);
            m_motor.set(ControlMode.PercentOutput,output);

            // Stop PID if within tolerance
            if (error < 0.05) {
                stopMotor();
                PIDOn = false;
            }
        }
    }

    public void stopMotor() {
        m_motor.set(ControlMode.PercentOutput, 0);
    }

    public void resetEncoder() {
        m_motor.setSelectedSensorPosition(0);
    }

    public double getEncoderDistance() {
        return m_motor.getSelectedSensorPosition();
    }

    public double getMotorSpeed() {
      return m_motor.getMotorOutputPercent();
    }
}