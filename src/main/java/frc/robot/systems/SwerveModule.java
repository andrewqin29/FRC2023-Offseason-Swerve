package frc.robot.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.CANCoder.CANCoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;



public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final PIDController turningPIDController;
    private final RelativeEncoder driveEncoder, turnEncoder;

    private final AnalogInput absoluteEncoder;
    private final boolean isAbsoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    //constants
    private static final double P_TURNING = 0.5;
    private static final double WHEEL_DIAMETER = Units.inchesToMeters(4); //assuming standard mk4 billet wheels
    private static final double TURNING_GEAR_RATIO = 1 / 12.8;
    private static final double DRIVE_GEAR_RATIO = 1 / 8.14; //assuming L1 gear ratio
    private static final double DRIVE_ENCODER_ROT2METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER; //1 drive motor rotation to meters
    private static final double TURNING_ENCODER_ROT2RAD = TURNING_GEAR_RATIO * 2 * Math.PI; //1 turning motor rotation to radians
    private static final double DRIVE_ENCODER_RPM2MPS = DRIVE_ENCODER_ROT2METER / 60;
    private static final double TURNING_ENCODER_RPM2RPS = TURNING_ENCODER_ROT2RAD / 60;
    private static final double MOTOR_PHYSICAL_MAX_SPEED_MPS = 12.0; //assuming L1 gear ratio, NEO unadjusted free speed
    
    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, boolean reversed, double offset) {

            //motors
            driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
            driveMotor.setInverted(driveMotorReversed);
            turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
            turnMotor.setInverted(turningMotorReversed);

            //encoders
            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getEncoder();

            driveEncoder.setPositionConversionFactor(DRIVE_ENCODER_ROT2METER);
            driveEncoder.setVelocityConversionFactor(DRIVE_ENCODER_RPM2MPS);
            turnEncoder.setPositionConversionFactor(TURNING_ENCODER_ROT2RAD);
            turnEncoder.setVelocityConversionFactor(TURNING_ENCODER_RPM2RPS);

            //absolute encoder
            absoluteEncoderOffset = offset;
            isAbsoluteEncoderReversed = reversed;
            absoluteEncoder = new AnalogInput(absoluteEncoderId);

            //pid turning controller
            turningPIDController = new PIDController(P_TURNING, 0, 0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoders();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public double getAbsoluteEncoderPosition() {
        //not implemented yet, figure it out
        return -1;
    }

    public void setState(SwerveModuleState state) {
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            driveMotor.set(0);
            turnMotor.set(0);
            return;
        }
        state = SwerveModuleState.optimize(state, getCurrentState().angle);
        driveMotor.set(state.speedMetersPerSecond/MOTOR_PHYSICAL_MAX_SPEED_MPS);
        turnMotor.set(turningPIDController.calculate(turnEncoder.getPosition(), state.angle.getRadians()));
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }
}
