package frc.robot.subsystems.Elevator;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSimSystem extends SubsystemBase implements ElevatorIO {
    ProfiledPIDController elevatorController = new ProfiledPIDController(2.3, 0, 0.2, new TrapezoidProfile.Constraints(12, 16));
    DCMotor elevatorGearbox = DCMotor.getNEO(2);
    PWMSparkMax elevatorMotor = new PWMSparkMax(10);
    Encoder elevatorEncoder = new Encoder(3, 4);

    EncoderSim elevatorEncoderSim = new EncoderSim(elevatorEncoder);

    double targetReference;
    ControlType currentControlType;

    private final ElevatorSim elevatorSim = new ElevatorSim(elevatorGearbox, 5, Units.lbsToKilograms(20), Units.inchesToMeters(1), 0, Units.inchesToMeters(70), true, 0);
    private final Mechanism2d elevatorSimMechanism = new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(58));
    private final MechanismRoot2d elevatorRoot = elevatorSimMechanism.getRoot("Base", Units.inchesToMeters(15), Units.inchesToMeters(8));
    public final MechanismLigament2d elevatorLigament = elevatorRoot.append(new MechanismLigament2d("Elevator", Units.inchesToMeters(10), 90, 10, new Color8Bit(Color.kBlue)));

    public ElevatorSimSystem() {
        elevatorController.reset(0);
        SmartDashboard.putData("Elevator Mech2D", elevatorSimMechanism);
        SmartDashboard.putNumber("Elevator Intake Height", ElevatorConstants.IntakeHeight);
        SmartDashboard.putNumber("Elevator L1 Height", ElevatorConstants.L1Height);
        SmartDashboard.putNumber("Elevator L2 Height", ElevatorConstants.L2Height);
        SmartDashboard.putNumber("Elevator L3 Height", ElevatorConstants.L3Height);
        SmartDashboard.putNumber("Elevator L4 Height", ElevatorConstants.L4Height);

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
        elevatorEncoder.setDistancePerPulse((2*Math.PI)/(1024*5));
        setPosition(22);
    }

    @Override
    public void periodic() {
        elevatorSim.setInput(elevatorMotor.get() * RobotController.getBatteryVoltage());
        elevatorSim.update(0.02);
        elevatorEncoderSim.setDistance(elevatorSim.getPositionMeters());
        elevatorLigament.setLength(elevatorSim.getPositionMeters() * 0.7 + 0.3);
        if (currentControlType == ControlType.kPosition) {
            double pidOutput = elevatorController.calculate(elevatorEncoder.getDistance() * 5, targetReference * (8.88/22));
            elevatorMotor.set(pidOutput);
        }

        SmartDashboard.putNumber("Elevator Position", getPosition());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
    
    @Override
    public void set(double speed) {
        elevatorMotor.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void setVelocity(double velocity) {
        elevatorController.setGoal(velocity);

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    @Override
    public void setPosition(double position) {
        SmartDashboard.putNumber("Requested Elevator Position", position);

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorMotor.setVoltage(voltage);
        
        currentControlType = ControlType.kVoltage;
    }

    @Override
    public void setEncoderPosition(double position) {
        elevatorSim.setState(position, 0);
    }

    @Override
    public double getVelocity() {
        return elevatorSim.getVelocityMetersPerSecond();
    }

    @Override
    public double getPosition() {      
        return elevatorEncoder.getDistance() * (22/1.75);
    }

    @Override
    public boolean atTarget(double threshold) {
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlType == ControlType.kPosition) {
            return Math.abs(getPosition() - targetReference) < threshold;
        } else {
            return false;
        }
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }

    @Override
    public Optional<MechanismLigament2d> returnLigament() {
        return Optional.of(elevatorLigament);
    }
}
