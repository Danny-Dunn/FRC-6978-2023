package frc.robot.Platform;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Hardware {
    //Gyro
    public static AHRS navX = new AHRS();

    //Drive Motors
    public static TalonFX leftDrive1 = new TalonFX(1);
    public static TalonFX leftDrive2 = new TalonFX(2);
    public static TalonFX rightDrive1 = new TalonFX(3);
    public static TalonFX rightDrive2 = new TalonFX(4);

    //Aft lift mechanism
    public static TalonFX liftMotor = new TalonFX(5);
    //Fwd cable lift
    public static TalonFX armCableMotor = new TalonFX(50);
    //Arm extension/retraction mechanism
    public static TalonSRX armSlideMotor = new TalonSRX(11);
    //Gripper wheels
    public static TalonSRX gripperWheelsMotor = new TalonSRX(24);

    //Pneumatics
    public static Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public static Solenoid driveGearShiftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    public static Solenoid gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    //Human input devices
    public static Joystick driverStick = new Joystick(0);
    public static Joystick operatorStick = new Joystick(1);


    public static void configureHardware() {
        //Drive motors
        rightDrive1.setInverted(true);
        rightDrive2.setInverted(true);

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
        
        liftMotor.config_kP(0, Constants.Arm.lift_kP);
        liftMotor.config_kI(0, Constants.Arm.lift_kI);
        liftMotor.config_kD(0, Constants.Arm.lift_kD);
    }
}
