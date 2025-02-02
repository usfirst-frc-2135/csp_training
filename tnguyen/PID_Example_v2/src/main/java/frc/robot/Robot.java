package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private final static double kDt = 0.020; // 20 msec per RoboRIO loop
    private final static double kv = 75.0; // Max velocity - RPS
    private final static double ka = 30.0; // Max acceleration
    private double goal = 4096;
    private boolean button2Pressed = false;
    private boolean button3Pressed = false;
    private XboxController controller = new XboxController(0);
    private ExampleSmartMotorController m_motor = new ExampleSmartMotorController(5);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

    private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    @Override
    public void robotInit() {
        m_motor.setPID(0.005, 0.0, 0.0);
        m_motor.resetEncoder();
    }

    @Override
    public void teleopPeriodic() {
        m_motor.resetEncoder();

        if (controller.getRawButton(2)) {
            // m_goal = new TrapezoidProfile.State(4096, 0);
            // button2Pressed = true;
            m_motor.updateMotorSpeed();// Start moving to goal 
        } else if (controller.getRawButton(3)) {
            m_goal = new TrapezoidProfile.State(0, 0);
            // button3Pressed = true;
            m_motor.stopMotor(); // Stop the motor
        } 

        // Create a motion profile based on the constraints and current state
        TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(kDt); // Calculate new setpoint

        // Update the motor controller with PID
        m_motor.setSetpoint(
            ExampleSmartMotorController.PIDMode.kPosition,
            m_setpoint.position,
            m_feedforward.calculate(m_setpoint.velocity) / 12.0
        );
        m_motor.updateMotorSpeed();
        
        

        // Display widgets on SmartDashboard
        SmartDashboard.putNumber("Setpoint", m_setpoint.position);
        SmartDashboard.putNumber("Current Position", m_motor.getEncoderDistance());
        SmartDashboard.putNumber("Goal", goal);
        SmartDashboard.putNumber("Error", m_motor.getError(4096,m_motor.getEncoderDistance()));
        SmartDashboard.putBoolean("Button 2 pressed", button2Pressed);
        SmartDashboard.putBoolean("Button 3 pressed", button3Pressed);
        SmartDashboard.putNumber("Motor Speed", m_motor.getMotorSpeed());
    }

    @Override
    public void disabledInit() {
        m_motor.setPID(0.5, 0.0, 0.0);
        m_motor.resetEncoder();
    }
}