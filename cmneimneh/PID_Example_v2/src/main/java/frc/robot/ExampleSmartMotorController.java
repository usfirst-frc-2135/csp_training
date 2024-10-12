package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;

public class ExampleSmartMotorController {
    private final WPI_TalonSRX m_motor;
    private double kp = 0.5;  // Proportional gain
    private double ki = 0.0;   // Integral gain
    private double kd = 0.0;   // Derivative gain

    private double prevError = 0;
    private double integral = 0;
    private double setpoint = 0;
    private boolean PIDOn = false;

    public ExampleSmartMotorController(int port) {
        m_motor = new WPI_TalonSRX(port);
        m_motor.config_kP(0, kp);
        m_motor.config_kI(0, ki);
        m_motor.config_kD(0, kd);
    }

    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
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

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        PIDOn = true; // Start PID control
    }

    public void updateMotorSpeed() {
        if (returnPIDOn() == true) {
            double currentPosition = m_motor.getSelectedSensorPosition();
            double error = setpoint - currentPosition;

            // Calculate integral and derivative
            integral += error;
            double derivative = error - prevError;

            // Calculate output
            double output = (kp * error) + (ki * integral) + (kd * derivative);
            m_motor.set(ControlMode.PercentOutput, output);

            // Update previous error
            prevError = error;

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
