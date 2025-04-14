package frc.robot.subsystems.Wrist;

import java.util.Optional;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSim extends SubsystemBase implements WristIO {
    ProfiledPIDController wristController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(8, 8));
    DCMotor wristGearbox = DCMotor.getNeoVortex(1);
    PWMSparkFlex wristMotor = new PWMSparkFlex(11);
    Encoder wristEncoder = new Encoder(0, 1);

    EncoderSim wristEncoderSim = new EncoderSim(wristEncoder);

    double targetReference;
    ControlType currentControlType;

    double jKgMS = SingleJointedArmSim.estimateMOI(Units.inchesToMeters(13), Units.lbsToKilograms(8));

    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(wristGearbox, 15, jKgMS, Units.inchesToMeters(13), -Math.PI/4, (5*Math.PI)/4, true, Math.PI/6);
    private final Mechanism2d wristSimMechanism = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(30));
    private final MechanismRoot2d wristHome = wristSimMechanism.getRoot("Base", Units.inchesToMeters(15), Units.inchesToMeters(15));
    public final MechanismLigament2d wristArm = wristHome.append(new MechanismLigament2d("Sword", Units.inchesToMeters(13), 0));

    public WristSim() {
        wristController.reset(0);
        SmartDashboard.putData("Wrist Mech2D", wristSimMechanism);
        SmartDashboard.putNumber("Wrist Intake Angle", WristConstants.IntakeAngle);
        SmartDashboard.putNumber("Wrist L1 Angle", WristConstants.L1Angle);
        SmartDashboard.putNumber("Wrist L2-3 Angle", WristConstants.L2_3Angle);
        SmartDashboard.putNumber("Wrist L4 Angle", WristConstants.L4Angle);

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
        wristEncoder.setDistancePerPulse((2*Math.PI)/(1024*25));
        setPosition(5);
    }

    @Override
    public void periodic() {
        wristSim.setInput(wristMotor.get() * RobotController.getBatteryVoltage());
        wristSim.update(0.02);
        wristEncoderSim.setDistance(wristSim.getAngleRads());
        wristArm.setAngle(270 + Math.toDegrees(Math.PI - wristSim.getAngleRads() % (2 * Math.PI) < 0 ? Math.PI - wristSim.getAngleRads() % (2 * Math.PI) + 2*Math.PI : Math.PI - wristSim.getAngleRads() % (2 * Math.PI)));
        if (currentControlType == ControlType.kPosition) {
            double pidOutput = wristController.calculate(wristSim.getAngleRads(), Units.rotationsToRadians(targetReference / 15));
            wristMotor.set(pidOutput);
        }

        SmartDashboard.putNumber("Wrist Position", Units.radiansToRotations(wristEncoder.getDistance()*15));

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
        SmartDashboard.putBoolean("Wrist at Position", Math.abs(getPosition() - targetReference) < 1);
    }
    
    @Override
    public void set(double speed) {
        wristMotor.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void setVelocity(double velocity) {
        wristController.setGoal(velocity);

        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    @Override
    public void setPosition(double position) {
        SmartDashboard.putNumber("Requested Wrist Position", position);

        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    @Override
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
        
        currentControlType = ControlType.kVoltage;
    }

    @Override
    public void setEncoderPosition(double position) {
        // Converts from wrist angle (rad) to motor rotations assuming 25:1 gear ratio
        wristSim.setState(position*2*Math.PI, 0);
    }

    @Override
    public double getVelocity() {
        return wristSim.getVelocityRadPerSec();
    }

    @Override
    public double getPosition() {      
        // Converts from wrist angle (rad) to motor rotations assuming 25:1 gear ratio  
        return Units.radiansToRotations(wristEncoder.getDistance()*15);
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
        return Optional.of(wristArm);
    }
}
