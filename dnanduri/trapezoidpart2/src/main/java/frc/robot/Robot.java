// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot { //Handles the periodic execution of code (such as auton)
  private final static double kv = 8.0; // Max velocity - RPS
  private final static double ka = 16.0; // Max acceleration - RPS^2
  private final static double goal_1 = 2;
  private final static double goal_2 = 0.0;
  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);

  private static double kDt = 0.02;

  private final XboxController controller = new XboxController(0);
  private final ExampleSmartMotorController a_extraMotor = new ExampleSmartMotorController(5);
  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile.Constraints m_Constraints =
      new TrapezoidProfile.Constraints(kv, ka);


  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  @Override
  public void robotInit() {
    a_extraMotor.setPID(0.5, 0.0, 0);
  }

  @Override
  public void teleopPeriodic() {
    a_extraMotor.resetEncoder();

    if (controller.getYButtonPressed()) {
      m_goal = new TrapezoidProfile.State(goal_1, 0); //If the Y button is pressed, the motor will spin until this position
    } else if (controller.getXButtonPressed()) {
      m_goal = new TrapezoidProfile.State(goal_2, 0); //If the X button is pressed, the motor will spin backwards to the original position
    }
    SmartDashboard.putNumber("goal point", m_goal.position);

    // if(a_extraMotor.getEncoderDistance() <= m_goal.position || a_extraMotor.getEncoderDistance() >= m_goal.position){
    //     a_extraMotor.stopMotor();
    //   }

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    //configures the motor controller to achieve a specific position using PID
    a_extraMotor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, m_feedforward.calculate(m_setpoint.velocity) / 12.0);
    
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("1-setpoint", m_setpoint.position);
    SmartDashboard.putNumber("velocity", m_setpoint.velocity);
  }
}