// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Robot extends TimedRobot {  
  private static double kDt = 0.02;
  /*private final static double kv = 2.0;
  private final static double ka = 8.0;
  private final static double goal_1 = 0.0;
  private final static double goal_2 = 0.5;
  */
  private final static double kv = 3.0;
  private final static double ka = 16.0;
  private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);


  private final XboxController m_controller = new XboxController(0);
  //private final ExampleSmartMotorController m_motor = new ExampleSmartMotorController(1);
  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.0);
  private final ExampleSmartMotorController m_TopMotor = new ExampleSmartMotorController(5);
  private final TrapezoidProfile.Constraints m_Constraints = new TrapezoidProfile.Constraints(75, 30);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
 // private final static TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(kv, ka);
  //private //final TrapezoidProfile m_profile =
      //new TrapezoidProfile(new TrapezoidProfile.Constraints(1.00, 0.05));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  @Override
  public void robotInit() {
    m_TopMotor.setPID(0.5, 0.0, 0.0);
    // Note: These gains are fake, and will have to be tuned for your robot.
    /**WPI_TalonSRX TopMotor = (WPI_TalonSRX) m_TopMotor;
    TopMotor.config_kP(0, 0.3); // set proportional gain
    TopMotor.config_kI(0, 0.05); // set integral gain
    TopMotor.config_kD(0, 0.7);*/
  }

  @Override
  public void teleopPeriodic() {
    m_TopMotor.resetEncoder();

    if (m_controller.getXButtonPressed()){
      m_goal = new TrapezoidProfile.State(0.5, 0);
      
    } else if (m_controller.getYButtonPressed()) {
      m_goal = new TrapezoidProfile.State(0, 0);
    }
      
    

    TrapezoidProfile profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    m_setpoint = profile.calculate(kDt);

    // Send setpoint to offboard controller PID
    m_TopMotor.setSetpoint(ExampleSmartMotorController.PIDMode.kPosition, m_setpoint.position, m_feedforward.calculate(m_setpoint.velocity) / 12.0);
  }   
  @Override
  public void robotPeriodic(){
    SmartDashboard.putNumber("goal point", m_goal.position);
    //SmartDashboard.putNumber("encoder position", m_TopMotor.getEncoderDistance());
    SmartDashboard.putNumber("velocity", m_setpoint.velocity);
  }
         
}
