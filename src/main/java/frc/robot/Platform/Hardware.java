package frc.robot.Platform;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Hardware {
    //Gyro
    public static AHRS navX = new AHRS();

    //Drive Motors
    public static WPI_TalonFX leftDrive1 = new WPI_TalonFX(1);
    public static WPI_TalonFX leftDrive2 = new WPI_TalonFX(2);
    public static WPI_TalonFX rightDrive1 = new WPI_TalonFX(3);
    public static WPI_TalonFX rightDrive2 = new WPI_TalonFX(4);

    //Aft lift mechanism
    public static WPI_TalonFX armLiftMotor = new WPI_TalonFX(5);
    //Fwd cable lift
    public static WPI_TalonFX armCableMotor = new WPI_TalonFX(50);
    //Arm extension/retraction mechanism
    public static WPI_TalonSRX armSlideMotor = new WPI_TalonSRX(11);
    //Gripper wheels
    public static WPI_TalonSRX gripperWheelsMotor = new WPI_TalonSRX(24);

    //Pneumatics
    public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public static Solenoid driveGearShiftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    public static Solenoid gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    //Human input devices
    public static Joystick driverStick = new Joystick(0);
    public static Joystick operatorStick = new Joystick(1);


    public static void configureHardware() {
        //Drive motors
        leftDrive1.setInverted(true);
        leftDrive2.setInverted(true);

        leftDrive2.set(ControlMode.Follower, 1);
        rightDrive2.set(ControlMode.Follower, 3);

        leftDrive1.config_kP(0, Constants.Drive.kP);
        leftDrive1.config_kI(0, Constants.Drive.kI);
        leftDrive1.config_kD(0, Constants.Drive.kD);

        rightDrive1.config_kP(0, Constants.Drive.kP);
        rightDrive1.config_kI(0, Constants.Drive.kI);
        rightDrive1.config_kD(0, Constants.Drive.kD);

        leftDrive1.setNeutralMode(NeutralMode.Coast);
        leftDrive2.setNeutralMode(NeutralMode.Coast);
        rightDrive1.setNeutralMode(NeutralMode.Coast);
        rightDrive2.setNeutralMode(NeutralMode.Coast);

        leftDrive1.configClosedLoopPeakOutput(0, 1);
        rightDrive1.configClosedLoopPeakOutput(0, 1);

        //Arm motors

        armLiftMotor.setSelectedSensorPosition(Constants.Arm.liftStartingPosition);

        armLiftMotor.config_kP(0, Constants.Arm.lift_kP);
        armLiftMotor.config_kI(0, Constants.Arm.lift_kI);
        armLiftMotor.config_kD(0, Constants.Arm.lift_kD);

        armCableMotor.setInverted(true);
        armCableMotor.setSelectedSensorPosition(Constants.Arm.cableStartingPosition);

        armCableMotor.config_kP(0, Constants.Arm.cable_kP);
        armCableMotor.config_kI(0, Constants.Arm.cable_kI);
        armCableMotor.config_kD(0, Constants.Arm.cable_kD);

        armSlideMotor.setInverted(true);
        armSlideMotor.setSensorPhase(true);
        armSlideMotor.setSelectedSensorPosition(Constants.Arm.slideStartingPosition);

        armSlideMotor.config_kP(0, Constants.Arm.slide_kP);
        armSlideMotor.config_kI(0, Constants.Arm.slide_kI);
        armSlideMotor.config_kD(0, Constants.Arm.slide_kD);

        //Pneumatics

        driveGearShiftSolenoid.set(Constants.Drive.gearShiftDefaultState);
        gripperSolenoid.set(Constants.Arm.gripperCloseDefaultState);

        //other
        navX.calibrate();
    }
}
