package frc.robot.subsystems;

import java.lang.module.Configuration;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem2  extends SubsystemBase{

    // Standard classes for controlling our elevator
    private final SparkMax m_motor = new SparkMax(10, MotorType.kBrushless);
    private final SparkMax f_motor = new SparkMax(11, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final PIDController m_controller = new PIDController(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd);
    private final DigitalInput  m_limitSwitchLow = new DigitalInput(0);
    private double m_setpoint = 0;

    /** Creates a new Drive Subsystem. */
    public ElevatorSubsystem2() {
       SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(ElevatorConstants.kMaxVelocity)
        .maxAcceleration(ElevatorConstants.kMaxAcceleration);
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        var output = m_controller.calculate(m_encoder.getPosition(), m_setpoint);
        m_motor.set(output);
        SmartDashboard.putNumber("Enc", m_encoder.getPosition());
        SmartDashboard.putNumber("ElevatorMotorDrive", output);
    }

    public void control(double rotation) {

        if (Math.abs(rotation) > Constants.OperatorConstants.DEADBAND) {
            m_setpoint += (rotation * 0.1);
        }
    }

    public Command elevatorHome() {
        m_setpoint = 0;
        if (m_limitSwitchLow.get()) {
            m_encoder.setPosition(0);
            m_motor.set(0);
        }
        return new PrintCommand("Elevator Homed");
    }

    public Command elevatorToHeight(double height) {
        m_setpoint = height;
        return new PrintCommand("Elevator to Height " + height);
    }
}
