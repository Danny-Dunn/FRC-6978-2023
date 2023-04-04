package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {

  //CANIFIER stuff for the LEDs
  CANifier canifier;
  double timeBetweenLightChanges = 0.2;
  double timeBetweenLightChangesFlag = 0.02;
  enum LightOption {
    off,
    weewoo,
    balance,
    cube,
    cone,
    disabled
  };

  LightOption myLightOption;

  double R = 0;
  double G = 0;
  double B = 0;

  TalonFX leftDrive1;
  TalonFX leftDrive2;
  TalonFX rightDrive1;
  TalonFX rightDrive2;

  TalonFX liftMotor;

  TalonSRX armWheels;
  TalonSRX armMotor;
  TalonFX armRotator;

  Joystick driveStick;
  Joystick operatorStick;

  AHRS navX;
  
  boolean isSticking;

  double angleP;
  double angleD;
  double roll;

  boolean brakeStatus;
  boolean armWheelsOn;

  Compressor compressor;
  Solenoid driveGearShiftSolenoid; //exitflag is gone :)
  Solenoid gripperSolenoid; //exitflag is gone :)
  boolean compressorState = false;
  boolean driveShiftBool = false;
  boolean gripperBool = true;
  boolean compState = true;

  //manual mode stuff
  boolean manualMode = true;

  //auto stuff
  int autoType = 0;
  double autoTimer = 0;


  //driving stuff
  boolean fastDriving = false;

  //setting the zero for the arm
  Timer timer;
  boolean zeroCompleted = false;
  boolean zeroSettingBOOL = false;
  double zeroSettingAmperage = 8;
  double zeroSettingTime = 0.2;
  double zeroSettingTimeFlag;

  double armMax;

  double armSlideGoal = 0;
  double armLiftGoal = 0;
  double armRotateGoal = 0;
  double armMaxDistance = 19000; //from testing
  double maxLiftSpeedDown = -0.1;
  double maxLiftSpeedUp = 0.3;

    //legend
    //0
    //cube placement mid
    //cone placement mid
    //cube placement high
    //cone placement high
    //floor pickup
    //pickup position
  /*double[] armRotatePositions = {0, 271000, 368000, 368000, 400000};
  double[] armLiftPositions = {0, -290000, -474000, -474000, -500000};
  double[] armSlidePositions = {0, 9700, 9000, 19000, 19000};*/
  //
  //37149

  double[] armRotatePositions = {0, 229174, 359000, 373000, 405000, 40149, 365000, -79753};
  double[] armLiftPositions = {0, -531184/2, -531000/2, -358000/2, -358000/2, 0, -375000/2, -640000/2};
  double[] armSlidePositions = {0, 0, 10051, 17000, 19900, 0, 0, 0};

  int armAutoPosition = 0;

  //this is the increment that the arm will go out by each time you click the button. ex 0 -> 10 -> 20 -> 10
  double armSlideGoalIncrement = 0.10;
  double liftGoalIncrement = 0.10;
  
  //lift pid 
  double time;
  double power;
  double shooterFlag;
  private double liftMaxPosition;
  private double armMaxPosition;
  private double liftP;
  private double liftI;
  private double liftD;
  private double liftSetpointPos;

  //safety
  double safetyTimerLift = 0;
  boolean liftAllowedToRun = true;
  boolean liftSafetyTriggered = false;
  double highestAmperage = 0;

  private double targetAngle;

  boolean slowTurning;

  //Collection<TalonFX> _fxes =  { new TalonFX(1), new TalonFX(2), new TalonFX(3) };

  Timer runTimer;

  boolean fmsConnected = false;
  boolean flashLights;
  double flashPeriod = 1;
  boolean redAlliance;

  //auto Variables
  int autoStep = 0;

  @Override
  public void robotInit() {
    runTimer = new Timer();
    runTimer.start();

    myLightOption = LightOption.off;
    canifier = new CANifier(40);

    leftDrive1 = new TalonFX(1);
    leftDrive2 = new TalonFX(2);
    rightDrive1 = new TalonFX(3);
    rightDrive2 = new TalonFX(4);
    liftMotor = new TalonFX(5);

    liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      false,
      0,
      5,
      1
    ));

    liftMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
      false,
      5,
      30,
      0.2
    ));

    //Arm Motor Talon
    //testMotor = new TalonSRX(24); //this was on the comp bot for the rotating arm
    armMotor = new TalonSRX(11);
    armMotor.setInverted(true);
    armMotor.setSensorPhase(false);
    armMotor.config_kP(0, 0.5);
    armMotor.configClosedloopRamp(0.2);
    //armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 12, ))

    //Arm Rotator Falcon
    armRotator = new TalonFX(50);
    armRotator.setInverted(false);

    //Arm Wheel/Intake Talon
    armWheels = new TalonSRX(24);
    armWheels.setInverted(true); //NEEDS TO BE CHANGED ON PRACTICE BOT

    //invert the right drives
    rightDrive1.setInverted(true);
    rightDrive2.setInverted(true);

    //set the second drive of each side to be a follower of the first drive
    leftDrive2.set(ControlMode.Follower, 1);
    rightDrive2.set(ControlMode.Follower, 3);

    //set all drive falcons to not break mode
    leftDrive1.setNeutralMode(NeutralMode.Coast);
    leftDrive2.setNeutralMode(NeutralMode.Coast);
    rightDrive1.setNeutralMode(NeutralMode.Coast);
    rightDrive2.setNeutralMode(NeutralMode.Coast); 

    leftDrive1.setSensorPhase(true);
    rightDrive1.setSensorPhase(true);

    leftDrive1.config_kP(0, 0.01);
    leftDrive1.config_kI(0, 0.0003);
    leftDrive1.config_kD(0, 1);

    leftDrive1.configClosedloopRamp(0.3);

    rightDrive1.config_kP(0, 0.01);
    rightDrive1.config_kI(0, 0.0003);
    rightDrive1.config_kD(0, 1);
    
    rightDrive1.configClosedloopRamp(0.3);



    leftDrive1.configClosedLoopPeakOutput(0, 1);
    rightDrive1.configClosedLoopPeakOutput(0, 1);

    driveStick = new Joystick(0);
    operatorStick = new Joystick(1);

    navX = new AHRS();

    navX.calibrate();

    navX.enableBoardlevelYawReset(true);
    navX.reset();

    isSticking = false;

    brakeStatus = true;

    //Pneumatics 
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    driveGearShiftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    SmartDashboard.putNumber("angleP", angleP);
    SmartDashboard.putNumber("angleD", angleD);

    timer = new Timer();
    timer.start();
    //lft PID
    liftP = 0.02; //100:1 lift motor
    //liftP = 0.01;
    liftMotor.config_kP(0, liftP);
    // liftI = 0.000002;
    liftMotor.config_kI(0, 0);
    // liftD = 23.0;
    liftMotor.config_kD(0, 0);
    // liftMotor.configOpenloopRamp(0.15);
    // liftMotor.setSelectedSensorPosition(0);
    // liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 8, 7, 0.05));
    // liftMotor.configClosedLoopPeakOutput(0, 0.5);
    
    // end of pid*/
    liftMaxPosition = -800000;
    armMaxPosition = 20000;

    //lift pid
    SmartDashboard.putNumber("Lift P", liftP);
    SmartDashboard.putNumber("Lift I", liftI);
    SmartDashboard.putNumber("Lift D", liftD);

    SmartDashboard.putNumber("Arm PID Increment %", 10);

    SmartDashboard.putNumber("ArmP", 0.181);   

    SmartDashboard.putNumber("shooter power (0-1)", 0.3); 

    SmartDashboard.putNumber("Auto Type: ", 0); 

    armRotator.setSelectedSensorPosition(0);
    armMotor.setSelectedSensorPosition(0);
    liftMotor.setSelectedSensorPosition(0);
    SmartDashboard.putBoolean("WheelieProtectionSticky", false);

  }

  @Override
  public void robotPeriodic() {
    double speed = 1;

    double intensity = 0.5;
    double amountGreen;
    double amountRed;
    double amountBlue; 

    switch(myLightOption){
      case disabled:
        if(redAlliance) {
          canifier.setLEDOutput(0, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(1, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(0, LEDChannel.LEDChannelC); //blue
        } else {
          canifier.setLEDOutput(0, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(0, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(1, LEDChannel.LEDChannelC); //blue
        }
        break;
      case off:
        speed = 1;

        intensity = 0.5;
    
        amountGreen = ((Math.sin((timer.get() * speed)) + 1) / 2) * intensity;
        amountRed = ((Math.sin((timer.get() * speed) + (Math.toRadians(90))) + 1) / 2) * intensity ;
        amountBlue = ((Math.sin((timer.get() * speed) + (Math.toRadians(180))) + 1) / 2) * intensity;
        
        canifier.setLEDOutput(amountGreen, LEDChannel.LEDChannelA); //green
        canifier.setLEDOutput(amountRed, LEDChannel.LEDChannelB); //red
        canifier.setLEDOutput(amountBlue, LEDChannel.LEDChannelC); //blue
      break;
      case weewoo:
        intensity = 0.5;
        if (G == 0 && timer.get() >= timeBetweenLightChangesFlag){
          timeBetweenLightChangesFlag = timer.get() + timeBetweenLightChanges;
          G = 1;
          canifier.setLEDOutput(0, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(1 * intensity, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(0, LEDChannel.LEDChannelC); //blue
        }else if (G == 1 && timer.get() >= timeBetweenLightChangesFlag){
          G = 0;
          timeBetweenLightChangesFlag = timer.get() + timeBetweenLightChanges;
          canifier.setLEDOutput(0, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(0, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(1 * intensity, LEDChannel.LEDChannelC); //blue
        }
      break;
      case balance:
        amountGreen = 1 - (Math.abs(navX.getPitch()) / 10);
        amountRed = (Math.abs(navX.getPitch()) / 10);
        amountBlue = 0;
        
        canifier.setLEDOutput(amountGreen, LEDChannel.LEDChannelA); //green
        canifier.setLEDOutput(amountRed, LEDChannel.LEDChannelB); //red
        canifier.setLEDOutput(amountBlue, LEDChannel.LEDChannelC); //blue
      break;
      case cube:
        if(timer.get() % flashPeriod > flashPeriod/2 |! flashLights) {
          canifier.setLEDOutput(0, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(1, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(1, LEDChannel.LEDChannelC); //blue
        } else {
          canifier.setLEDOutput(0, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(0.25, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(0.25, LEDChannel.LEDChannelC); //blue
        }
      break;
      case cone:
        if(timer.get() % flashPeriod > flashPeriod/2 |! flashLights) {
          canifier.setLEDOutput(0.7, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(1, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(0, LEDChannel.LEDChannelC); //blue
        } else {
          canifier.setLEDOutput(0.175, LEDChannel.LEDChannelA); //green
          canifier.setLEDOutput(0.25, LEDChannel.LEDChannelB); //red
          canifier.setLEDOutput(0, LEDChannel.LEDChannelC); //blue
        }
      break;
    }

    if (operatorStick.getPOV() == 270){
      myLightOption = LightOption.cube;
      flashLights = true;
    } 
    else if (operatorStick.getPOV() == 90){
      myLightOption = LightOption.cone;
      flashLights = true;
    } 
    else if (driveStick.getRawButton(13)){
      myLightOption = LightOption.weewoo;
    } 
    else if (driveStick.getRawButton(14)){
      myLightOption = LightOption.balance;
    } else if(operatorStick.getPOV() == 0){
      myLightOption = LightOption.off;
    } else {
      flashLights = false;
    }

    CommandScheduler.getInstance().run();

    //SmartDashboard.putNumber("shooter runtime", 2); :)
    power = SmartDashboard.getNumber("shooter power (0-1)", 0.3);
    time = SmartDashboard.getNumber("shooter runtime", 2);

    angleP = SmartDashboard.getNumber("angleP", 0.00001);
    angleD = SmartDashboard.getNumber("angleD", 0);
    roll = navX.getPitch();

    SmartDashboard.putBoolean("BreakStatus", brakeStatus);
    SmartDashboard.putNumber("pitch", roll);
    SmartDashboard.putBoolean("sticking?", isSticking);

    SmartDashboard.putNumber("Arm Length POS", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Length AMP", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("Arm Length MAX", armMax);
    SmartDashboard.putNumber("Arm length applied", armMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Arm Rotation Position", armRotator.getSelectedSensorPosition()); //400,000 to 0
    
    //SmartDashboard.putNumber("Arm Rot POS", armRotator.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift Position", liftMotor.getSelectedSensorPosition()); //-800,000 to 0

    SmartDashboard.putNumber("Arm PID GOAL", armSlideGoal);
    armSlideGoalIncrement = SmartDashboard.getNumber("Arm PID Increment %", 10)/100;
    liftGoalIncrement = SmartDashboard.getNumber("Lift PID Increment %", 10)/100;

    SmartDashboard.putBoolean("ManualMode", manualMode);
    if(operatorStick.getRawButtonPressed(9)){
      manualMode = !manualMode; 
    }

    SmartDashboard.putNumber("LeftDriveVEL", leftDrive1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RightDriveVEL", rightDrive1.getSelectedSensorVelocity());

    SmartDashboard.putNumber("ArmAutoPosition", armAutoPosition);

    SmartDashboard.putNumber("navX angle", navX.getAngle());
    SmartDashboard.putNumber("navX yaw", navX.getYaw());
    SmartDashboard.putNumber("target angle", targetAngle);
    SmartDashboard.putNumber("Lift Current", liftMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Lift MAX Current", highestAmperage);

    //SmartDashboard.putNumber("ArmP", 0);

    SmartDashboard.putBoolean("Lift Allowed ON:", liftAllowedToRun);

    SmartDashboard.putBoolean("High Gear", driveShiftBool);

    liftSafetyShutoff(0.1, 10);

    if (liftMotor.getSupplyCurrent() > highestAmperage){
      highestAmperage = liftMotor.getSupplyCurrent();
    }

    

    //SmartDashboard.putNumber("Auto Step", autoStep);

    SmartDashboard.putNumber("LeftDrive POS", leftDrive1.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightDrive POS", rightDrive1.getSelectedSensorPosition());

    SmartDashboard.putNumber("Yaw", navX.getAngle());

    SmartDashboard.putBoolean("Slow Turning", slowTurning);

    //look for alliance status for 30 seconds after 
    if(runTimer.get() < 30) {
      redAlliance = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    myLightOption = LightOption.disabled;
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //Set rotate, lift and slide to 0
    armRotator.setSelectedSensorPosition(0);
    armMotor.setSelectedSensorPosition(0);
    liftMotor.setSelectedSensorPosition(0);

    //reset navx
    navX.reset();

    autoType = (int)SmartDashboard.getNumber("Auto Type: ", 0); 

    //Set drive positions to 0
    leftDrive1.setSelectedSensorPosition(0);
    rightDrive1.setSelectedSensorPosition(0);

    liftAllowedToRun = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /* POTNENTIAL AUTO STEPS
     * Placing a block
     *  - Don't place a block (1)
     *  - bottom layer: push it in, regardless of piece type (2)
     *  - mid layer with cone: move lift up, extend arm a bit and release claw (3)
     *  - mid layer with cube: move lift up, extend arm a bit and drive arm wheels out (4)
     *  - top layer with cone: move lift up, rotate arm up, extend arm and release claw (5)
     *  - top layer with cube: move lift up, rotate arm up, extend arm and drive arm wheels out (6)
     * Moving out of community
     *  - Starting in middle of field: drive over the switch, then come back to balance (1)
     *  - starting near sides of field: drive around switch and come back to balance (2)
     * Balancing
     *  - Balance (1)
     *  - Don't balance, teammate is 1114 (2)
     * 
     * - We can create different combinations of auto steps, and call them based on the number 
     * (ex. 2-2-1 auto is scoring a bottom layer piece, driving around the switch and then balancing from the other side)
     */
    switch(autoType){
      case 1:
        auto1_DropPieceMoveBackwardNoBalanceLeftSide();
        break;
      case 2:
        auto2_DropPieceMoveBackwardBalanceBackward();
        break;
      case 3:
        auto1_DropPieceMoveBackwardNoBalanceRightSide();
        break;
      case 4:
        auto4_SwitchBalance();
      default:
        break;
    }

    if (!liftAllowedToRun){
      liftMotor.set(ControlMode.Disabled, 0);
    }
    
  }

  //Drop cone, move backwards, rotate -173 degrees
  public void auto1_DropPieceMoveBackwardNoBalanceLeftSide(){
    switch(autoStep){
      case 0:
        if (armRotator.getSelectedSensorPosition() < armRotatePositions[1] * 0.9){
          armRotator.set(ControlMode.Position, armRotatePositions[1]);
          if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[1]);
          armMotor.set(ControlMode.PercentOutput, -0.1);
          gripperSolenoid.set(false);
        }
        else{
          autoStep++;
          armMotor.setSelectedSensorPosition(0);
        }
        break;
        case 1:
        armRotator.set(ControlMode.Position, armRotatePositions[2]);
        autoStep++;
        break;
      case 2:
        if (armRotator.getSelectedSensorPosition() > armRotatePositions[2] * 0.92){
          armMotor.set(ControlMode.Position, armSlidePositions[2]);
          //armPID(armSlidePositions[2]);
          if (liftAllowedToRun) {liftMotor.set(ControlMode.Position, armLiftPositions[2]);}
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 3:
        if (timer.get() - autoTimer > 1.25){
          gripperSolenoid.set(true);
          autoTimer = timer.get();
          autoStep++;
        }
        break;
      case 4:
        if (timer.get() - autoTimer > 1){
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 5:
        //drive backwards and lower into floor pickup position
        if (timer.get() - autoTimer > 1){
          armMotor.set(ControlMode.Position, armSlidePositions[7]);
          armRotator.set(ControlMode.Position, armRotatePositions[7]);
          if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[7]);
        }
        
        if (autoDriveToPositionVelocityDrive(320000, 8000, 9000, 0.0001)){
          autoStep++;
        }
        break;
      case 6:
        if (autoTurnToAngleVelocityDrive(-173, 4500, 0.04)){
          autoStep++;
          leftDrive1.set(ControlMode.Disabled, 0);
          rightDrive1.set(ControlMode.Disabled, 0);
          leftDrive1.setSelectedSensorPosition(0);
          rightDrive1.setSelectedSensorPosition(0);
          armWheels.set(ControlMode.PercentOutput, 0.24);
          armAutoPosition = 5;
        }
        break;
      case 7:
        if(autoDriveToPositionVelocityDrive(-80000, 3000, 4000, 0.0001)) {
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 8:
        armMotor.set(ControlMode.Position, armSlidePositions[5]);
        armRotator.set(ControlMode.Position, armRotatePositions[5]);
        if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[5]);
        if (timer.get() - autoTimer > 0.7){
          autoStep++;
        }
        break;
      case 9:
        if(autoTurnToAngleVelocityDrive(0, 4500, 0.04)) {
          autoStep++;
          leftDrive1.set(ControlMode.Disabled, 0);
          rightDrive1.set(ControlMode.Disabled, 0);
          leftDrive1.setSelectedSensorPosition(0);
          rightDrive1.setSelectedSensorPosition(0);
        }
        break;
      case 10:
        if(autoDriveToPositionVelocityDrive(-80000, 3000, 5500, 0.0001)) {
          autoStep++;
        }
        break;
    }
  }

  //Drop cone, move backwards, rotate 167 degrees
  public void auto1_DropPieceMoveBackwardNoBalanceRightSide(){
    switch(autoStep){
      case 0:
        if (armRotator.getSelectedSensorPosition() < armRotatePositions[1] * 0.9){
          armRotator.set(ControlMode.Position, armRotatePositions[1]);
          if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[1]);
          armMotor.set(ControlMode.PercentOutput, -0.1);
          gripperSolenoid.set(false);
        }
        else{
          autoStep++;
          armMotor.setSelectedSensorPosition(0);
        }
        break;
        case 1:
        armRotator.set(ControlMode.Position, armRotatePositions[2]);
        autoStep++;
        break;
      case 2:
        if (armRotator.getSelectedSensorPosition() > armRotatePositions[2] * 0.92){
          armMotor.set(ControlMode.Position, armSlidePositions[2]);
          //armPID(armSlidePositions[2]);
          if (liftAllowedToRun) {liftMotor.set(ControlMode.Position, armLiftPositions[2]);}
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 3:
        if (timer.get() - autoTimer > 1){
          gripperSolenoid.set(true);
          autoTimer = timer.get();
          autoStep++;
        }
        break;
      case 4:
        if (timer.get() - autoTimer > 1){
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 5:
        //drive backwards and lower into floor pickup position
        if (timer.get() - autoTimer > 1){
          armMotor.set(ControlMode.Position, armSlidePositions[7]);
          armRotator.set(ControlMode.Position, armRotatePositions[7]);
          if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[7]);
        }
        
        if (autoDriveToPositionVelocityDrive(320000, 8000, 10000, 0.0001)){
          autoStep++;
        }
        break;
      case 6:
        if (autoTurnToAngleVelocityDrive(167, 4500, 0.04)){
          autoStep++;
          leftDrive1.set(ControlMode.PercentOutput, 0);
          rightDrive1.set(ControlMode.PercentOutput, 0);
          leftDrive1.setSelectedSensorPosition(0);
          rightDrive1.setSelectedSensorPosition(0);
          armWheels.set(ControlMode.PercentOutput, 0.25);
          armAutoPosition = 5;
        }
        break;
        case 7:
        if(autoDriveToPositionVelocityDrive(-80000, 3000, 4000, 0.0001)) {
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 8:
        armMotor.set(ControlMode.Position, armSlidePositions[5]);
        armRotator.set(ControlMode.Position, armRotatePositions[5]);
        if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[5]);
        if (timer.get() - autoTimer > 0.7){
          autoStep++;
        }
        break;
      case 9:
        if(autoTurnToAngleVelocityDrive(0, 4500, 0.04)) {
          autoStep++;
          leftDrive1.set(ControlMode.Disabled, 0);
          rightDrive1.set(ControlMode.Disabled, 0);
          leftDrive1.setSelectedSensorPosition(0);
          rightDrive1.setSelectedSensorPosition(0);
        }
        break;
      case 10:
        if(autoDriveToPositionVelocityDrive(-80000, 3000, 5500, 0.0001)) {
          autoStep++;
        }
        break;
    }
  }

  //Drop cone, move backwards, balance on charge station
  public void auto2_DropPieceMoveBackwardBalanceBackward(){
    switch(autoStep){
      case 0:
        if (armRotator.getSelectedSensorPosition() < armRotatePositions[2] * 0.9){
          armRotator.set(ControlMode.Position, armRotatePositions[2]);
          if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[1]);
          armMotor.set(ControlMode.PercentOutput, -0.1);
          gripperSolenoid.set(false);
        }
        else{
          autoStep++;
          armMotor.setSelectedSensorPosition(0);
        }
        break;
      case 1:
        armRotator.set(ControlMode.Position, armRotatePositions[2]);
        autoStep++;
        break;
      case 2:
        if (armRotator.getSelectedSensorPosition() > armRotatePositions[2] * 0.9){
          armMotor.set(ControlMode.Position, armSlidePositions[2]);
          //armPID(armSlidePositions[2]);
          if (liftAllowedToRun) {liftMotor.set(ControlMode.Position, armLiftPositions[2]);}
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 3:
        if (timer.get() - autoTimer > 1){
          gripperSolenoid.set(true);
          autoTimer = timer.get();
          autoStep++;
        }
        break;
      case 4:
        if (timer.get() - autoTimer > 1){
          autoStep++;
        }
        break;
      case 5:
        //drive backwards and lower into floor pickup position
        armMotor.set(ControlMode.Position, armSlidePositions[0]);
        armRotator.set(ControlMode.Position, armRotatePositions[0]);
        if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[0]);
        driveShiftBool = false;
        setBrakeMode(driveShiftBool);
        leftDrive1.set(ControlMode.Velocity, 5600);
        rightDrive1.set(ControlMode.Velocity, 5600);
        autoStep++;
        break;
      case 6:
        if (autoDriveToPositionVelocityDrive(180000, 6500, 6500, 0.0001)){
        //if (navX.getPitch() < -8){
          //leftDrive1.set(ControlMode.Disabled, 0);
          //rightDrive1.set(ControlMode.Disabled, 0);
          autoStep++;
          
          autoTimer = timer.get();
          myLightOption = LightOption.balance;
        }
        break;
      case 7:
      
        //if (timer.get() - autoTimer > 1){
          setBrakeMode(true);
          balanceRobot_DrivingBackward(true);
        //}
        break;
    }
  }

  //Drop cone, move backwards over the charging station past community line, balance
  public void auto4_SwitchBalance(){
    switch(autoStep){
      case 0:
        if (armRotator.getSelectedSensorPosition() < armRotatePositions[2] * 0.9){
          armRotator.set(ControlMode.Position, armRotatePositions[2]);
          if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[1]);
          armMotor.set(ControlMode.PercentOutput, -0.1);
          gripperSolenoid.set(false);
        }
        else{
          autoStep++;
          armMotor.setSelectedSensorPosition(0);
        }
        break;
      case 1:
        armRotator.set(ControlMode.Position, armRotatePositions[2]);
        autoStep++;
        break;
      case 2:
        if (armRotator.getSelectedSensorPosition() > armRotatePositions[2] * 0.9){
          armMotor.set(ControlMode.Position, armSlidePositions[2]);
          //armPID(armSlidePositions[2]);
          if (liftAllowedToRun) {liftMotor.set(ControlMode.Position, armLiftPositions[2]);}
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 3:
        if (timer.get() - autoTimer > 1){
          gripperSolenoid.set(true);
          autoTimer = timer.get();
          autoStep++;
        }
        break;
      case 4:
        if (timer.get() - autoTimer > 1){
          autoStep++;
        }
        break;
      case 5:
        //drive backwards and lower into floor pickup position
        
        driveShiftBool = false;
        setBrakeMode(driveShiftBool);
        autoStep++;
        break;
      case 6:
      if (timer.get() - autoTimer > 1){
        armMotor.set(ControlMode.Position, armSlidePositions[0]);
        armRotator.set(ControlMode.Position, armRotatePositions[0]);
        if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[0]);
      }
        if (autoDriveToPositionVelocityDrive(360000, 6500, 6500, 0.0001)){
        //if (navX.getPitch() < -8){
          //leftDrive1.set(ControlMode.Disabled, 0);
          //rightDrive1.set(ControlMode.Disabled, 0);
          autoStep++;
          
          autoTimer = timer.get();
        }
        break;
      case 7:
        if (autoDriveToPositionVelocityDrive(180000, 6500, 6500, 0.0001)){
        //if (navX.getPitch() < -8){
          //leftDrive1.set(ControlMode.Disabled, 0);
          //rightDrive1.set(ControlMode.Disabled, 0);
          autoStep++;
          myLightOption = LightOption.balance;
          autoTimer = timer.get();
        }
        break;
      case 8:
      
        //if (timer.get() - autoTimer > 1){
          setBrakeMode(true);
          balanceRobot_DrivingBackward(true);
        //}
        break;
    }
  }

  public boolean autoDriveToPositionVelocityDrive(int distanceGoal, int goalVelocity, int maxOut, double p){
    double error = distanceGoal - ((leftDrive1.getSelectedSensorPosition() + rightDrive1.getSelectedSensorPosition()) / 2);
    double output = error * goalVelocity * p;

    output = output > maxOut ? maxOut : output;
    output = output < -maxOut ? -maxOut : output;

    leftDrive1.set(ControlMode.Velocity, output);
    rightDrive1.set(ControlMode.Velocity, output);

    if (Math.abs(leftDrive1.getSelectedSensorPosition() - distanceGoal) < 5000){
      return true;
    }
    return false;
  }

  public boolean autoTurnToAngleVelocityDrive(double angleGoal, double maxOut, double p){
    double yaw = navX.getAngle();

    double error = angleGoal - yaw;
    double output = error * p * maxOut;

    output = output > maxOut ? maxOut : output;
    output = output < -maxOut ? -maxOut : output;

    leftDrive1.set(ControlMode.Velocity, -output);
    rightDrive1.set(ControlMode.Velocity, output);

    if (Math.abs(yaw - angleGoal) < 2){
      return true;
    }
    return false;
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    double armP = SmartDashboard.getNumber("ArmP", 0);

    //armMotor.config_kP(0, 0.02);
    //armMotor.config_kD(0, 0);
    brakeStatus = false;
    setBrakeMode(false);

    liftAllowedToRun = true;
    manualMode = true;

    //armMotor.setSelectedSensorPosition(0);

    armRotator.config_kP(0, 0.013);
    armRotator.config_kI(0, 0.0);
    armRotator.config_kD(0, 0.0);

    leftDrive1.setSelectedSensorPosition(0);
    rightDrive1.setSelectedSensorPosition(0);

    SmartDashboard.putBoolean("WheelieProtectionSticky", false);

    myLightOption = LightOption.off;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    slowTurning = driveStick.getRawButton(5);

    if(driveStick.getRawButton(30)){
      
      if(Math.abs(navX.getRoll()) >= 3){
        isSticking = true;
      } 
      
      if(isSticking){
        balancingPID(-0.016, angleD, 0.4);
      } else {
        leftDrive1.set(ControlMode.PercentOutput, 0.4);
        rightDrive1.set(ControlMode.PercentOutput, 0.4);
      }
    }
    else {
      if(driveStick.getRawButtonPressed(14)) {
        targetAngle = calculateAngleToTurn(180);
      }
      if (driveStick.getRawButton(14)){
        balanceRobot_DrivingBackward(false);
        setBrakeMode(true);
        driveShiftBool = false;
        //yawPID(0.037, targetAngle, 2500);
      }
      else{
        driveButBetter();
        //drive();
      }
      isSticking = false;
      if (brakeStatus && !isSticking){
        brakeStatus = false;
        //setBrakeMode();
      }

    }

    ////////////////////////
    //pneumatics 
    /////////////

    //gear shifting
    if(driveStick.getRawButtonPressed(10)){
      driveShiftBool = !driveShiftBool;

      leftDrive1.configClosedloopRamp(driveShiftBool? 0.5:0.3);
      rightDrive1.configClosedloopRamp(driveShiftBool? 0.5:0.3);
    }
    driveGearShiftSolenoid.set(driveShiftBool);

    //Gripper controls
    if(operatorStick.getRawButtonPressed(4)){
      gripperBool = !gripperBool;
    }
    gripperSolenoid.set(gripperBool);
     
    //Arm wheels/shooter wheels
    //Get flag
    if(driveStick.getRawButtonPressed(3)){
      shooterFlag = timer.get() + time;
    }
    //Shooter wheel control
    if(driveStick.getRawButton(3)){ //out
      shooter();
      armWheelsOn = false;
     }else if(driveStick.getRawButton(4)){ //in a lot
      armWheels.set(ControlMode.PercentOutput, 0.8);
      armWheelsOn = false;
     }else if(driveStick.getRawButton(1)){ //in a little bit
        armWheels.set(ControlMode.PercentOutput, 0.25);
        armWheelsOn = true;
     }else{
      if (armWheelsOn){
        armWheels.set(ControlMode.PercentOutput, 0.25);
      }
        else{
          armWheels.set(ControlMode.PercentOutput, 0);
        }
      }

    //Set lift, rotator and extender values to 0
    if (operatorStick.getRawButtonReleased(10)){
      armMotor.setSelectedSensorPosition(0);
      armRotator.setSelectedSensorPosition(0);
      liftMotor.setSelectedSensorPosition(0);
      armAutoPosition = 0;
    }

    if (manualMode){
      //Move arm in or out
      if (operatorStick.getRawButton(6)){ //arm out
        armMotor.set(ControlMode.PercentOutput, 0.4);
      }
      else if (operatorStick.getRawButton(5)){ //arm in
        armMotor.set(ControlMode.PercentOutput, -0.4);
      }
      else{
        armMotor.set(ControlMode.PercentOutput, 0);
      }
      
      //Move lift up or down using right stick
      liftMotor.set(ControlMode.PercentOutput, -operatorStick.getRawAxis(3)/2);

      //Move rotator up or down using the left stick
      armRotator.set(ControlMode.PercentOutput, -Math.pow(operatorStick.getRawAxis(1), 3));
    }
    else{ //not in manual mode
      //Arm, lift and extender positions
      if(operatorStick.getRawButton(5)){
        armAutoPosition = 1; // cube mid
      }
      if(armAutoPosition == 1 || armAutoPosition == 5) {
        if(operatorStick.getRawButton(1)){
          armAutoPosition = 0; //park
        }else if(operatorStick.getRawButton(6)){
          armAutoPosition = 2; //cone mid
        }
        else if(operatorStick.getRawButton(7)){
          armAutoPosition = 3; //cube high
        }
        else if(operatorStick.getRawButton(8)){
          armAutoPosition = 4; //cone high
        }
        else if(operatorStick.getPOV() == 180){
          armAutoPosition = 5; // partial park
        } else if (operatorStick.getRawButton(3)) {
          armAutoPosition = 6; // floor pickup
        } else if (operatorStick.getRawButton(2)) {
          armAutoPosition = 7; // station pickup
        }
      }

      //Set lift, rotator and extender to positions given above
      armMotor.set(ControlMode.Position, armSlidePositions[armAutoPosition]);
      liftMotor.set(ControlMode.Position, armLiftPositions[armAutoPosition]);
      armRotator.set(ControlMode.Position, armRotatePositions[armAutoPosition]);
    }
  }

  //drive backwards balancing 
  public void balanceRobot_DrivingBackward(boolean halfSpeed){
    double curPitch = Math.sin(Math.toRadians(navX.getPitch()));
    double p = 2;

    double output = -curPitch * p;

    int maxOut = 2000;
    if (halfSpeed) maxOut = 1000;

    output = output * maxOut;

    output = output > maxOut ? maxOut : output;
    output = output < -maxOut ? -maxOut : output;

    leftDrive1.set(ControlMode.Velocity, output);
    rightDrive1.set(ControlMode.Velocity, output);
  }

  //Arm PID 
  public void armPID(double Goal){
    double error = Goal - armMotor.getSelectedSensorPosition();
    double p = 0.00025;
    double output = error*p;

    double max = 1;
    output = output > max ? max: output;
    output = output < -max ? -max: output;

    armMotor.set(ControlMode.PercentOutput, output);
  }

  //Lift PID
  public void liftPID(double percentGoal){
    double goal = liftMaxPosition * percentGoal/100;
    // double error = goal - liftMotor.getSelectedSensorPosition();
    // double liftP = 0.0001;
    // double output = error * liftP;
    // if(output > maxLiftSpeedUp){
    //   output = maxLiftSpeedUp;
    // }else if(output < maxLiftSpeedDown){
    //   output = maxLiftSpeedDown;
    // }
    if (liftAllowedToRun) liftMotor.set(ControlMode.Position, goal);
  }

  //Arm rotate PID
  public void armRotatePID(double percentGoal){
    double goal = armMaxPosition * percentGoal/100;
    
    armRotator.set(ControlMode.Position, goal);
  }

  //Zero the arm
  public void zeroArm(boolean findingMin){
    if (Math.abs(armMotor.getStatorCurrent()) > zeroSettingAmperage){
      if (zeroSettingBOOL == false){
        zeroSettingTimeFlag = timer.get() + zeroSettingTime;
      }
      zeroSettingBOOL = true;
    }
    else{
      zeroSettingTimeFlag = Double.MAX_VALUE;
      zeroSettingBOOL = false;
    }
    if (zeroSettingBOOL){
      if (timer.get() > zeroSettingTimeFlag){
        zeroCompleted = true;
        if (findingMin){
          armMotor.setSelectedSensorPosition(0);
        }
        else{
          armMax = armMotor.getSelectedSensorPosition();
        }
      }
    }
    if (!zeroCompleted){
      if (findingMin){
        armMotor.set(ControlMode.PercentOutput,-0.2);
      }
      else{
        armMotor.set(ControlMode.PercentOutput, 0.2);
      }
    }
    else{
      armMotor.set(ControlMode.PercentOutput,0);
    }
  }

  //Set brake mode for the drives
  public void setBrakeMode(boolean brake){
    if (brake){
      leftDrive1.setNeutralMode(NeutralMode.Brake);
      leftDrive2.setNeutralMode(NeutralMode.Brake);
      rightDrive1.setNeutralMode(NeutralMode.Brake);
      rightDrive2.setNeutralMode(NeutralMode.Brake);
    }
    else{
      leftDrive1.setNeutralMode(NeutralMode.Coast);
      leftDrive2.setNeutralMode(NeutralMode.Coast);
      rightDrive1.setNeutralMode(NeutralMode.Coast);
      rightDrive2.setNeutralMode(NeutralMode.Coast);
    }
  }
  
  //Auto Distance PID
  public void distancePID(double goal, double P, double maxSpeed){
    double position = (leftDrive1.getSelectedSensorPosition() + rightDrive1.getSelectedSensorPosition()) / 2;
    double error = goal - position;

    double output = error * P * maxSpeed;

    leftDrive1.set(ControlMode.Velocity, output);
    rightDrive1.set(ControlMode.Velocity, output);

    if(error <= goal - 1000){
      leftDrive1.set(ControlMode.Disabled, 0);
      rightDrive1.set(ControlMode.Disabled, 0);
    }
  }

  //Balancing PID
  public void balancingPID(double P, double D, double max){
    double error = navX.getRoll();
    double demand = error * P;

    demand = (demand > max)? max:demand;
    demand = (demand < -max)? -max:demand;

    leftDrive1.set(ControlMode.PercentOutput, demand);
    rightDrive1.set(ControlMode.PercentOutput, demand);
  }

  // Detecting angle change 
  public double yawPID(double P, double targetAngle, double maxSpeed) {
    double error = navX.getAngle() - targetAngle;
    double demand = error * P * maxSpeed;

    demand = (demand > maxSpeed)? maxSpeed : demand;
    demand = (demand < -maxSpeed)? -maxSpeed : demand;

    SmartDashboard.putNumber("yaw target velocity", demand);

    leftDrive1.set(ControlMode.Velocity, demand);
    rightDrive1.set(ControlMode.Velocity, -demand);
    return error;
  }

  //Calculating angle when turning
  public double calculateAngleToTurn(double desiredYaw) {
    double realYaw = navX.getAngle();
    double absYaw = angleToAbsYaw(realYaw);
    double absDelta;

    //determine if a left or right turn is more efficient
    absDelta = desiredYaw - absYaw;
    if(absDelta > 180) {
      absDelta = absDelta - 360;
    } else if (absDelta < -180) {
      absDelta = absDelta + 360;
    }
    return absDelta + realYaw;
  }

  public double angleToAbsYaw(double angle) {
    return (angle%360);
  }

  //Driving with PercentOutput
  public void drive(){
    double x;
    double y;
    double maxY = 1;

    //Negative X for 2023 practice bot, positive X for 2022 comp bot
    x = -driveStick.getRawAxis(0);
    y = (driveStick.getRawAxis(3) + 1 )/2 - (driveStick.getRawAxis(4) + 1 )/2 ;

    x = x * x *x; 
    if(y > maxY){
      y = maxY;
    }else if(y < -maxY){
      y = -maxY;
    }
    leftDrive1.set(ControlMode.PercentOutput, y + x );
    rightDrive1.set(ControlMode.PercentOutput, y - x);
  }

  //Driving with Velocity
  public void driveButBetter(){
    double x;
    double y;
    double maxY = 1;
    //max goal = -20000
    double goal = 20000;
    if(driveShiftBool) {
      goal = 12500;
    }

    double xdivisor = (slowTurning)? 6 : 3;

    //Negative X for 2023 practice bot, positive X for 2022 comp bot
    x = -driveStick.getRawAxis(0) / xdivisor;
    y = (driveStick.getRawAxis(3) + 1 )/2 - (driveStick.getRawAxis(4) + 1 )/2 ;

    //x = x * x *x; 
    if(y > maxY){
      y = maxY;
    }else if(y < -maxY){
      y = -maxY;
    }

    x = (Math.abs(x) < 0.1)? 0 : x;
    x = (x > 0.1)? ((x-0.1)/0.9) : x;
    x = (x < -0.1)? ((x+0.1)/0.9) : x;

    y = (Math.abs(y) < 0.1)? 0 : y;
    y = (y > 0.1)? ((y-0.1)/0.9) : y;
    y = (y < -0.1)? ((y+0.1)/0.9) : y;

    //y *= 1 - (navX.getPitch() * 0.1);

    SmartDashboard.putNumber("DriveX", x);
    SmartDashboard.putNumber("DriveY", y);

    SmartDashboard.putBoolean("WheelieProtection", false);

    /*if(Math.abs(navX.getPitch()) > 2.0 && driveShiftBool) {
      SmartDashboard.putBoolean("WheelieProtectionSticky", true);
      SmartDashboard.putBoolean("WheelieProtection", true);
      //leftDrive1.set(ControlMode.Disabled, 0);
      //rightDrive1.set(ControlMode.Disabled, 0);
    }
    else*/ if(x == 0 && y == 0) {
      leftDrive1.set(ControlMode.Disabled, 0);
      rightDrive1.set(ControlMode.Disabled, 0);
    } else {
      leftDrive1.set(ControlMode.Velocity, (y + x)*goal );
      rightDrive1.set(ControlMode.Velocity, (y - x)*goal);
    }

    SmartDashboard.putNumber("LeftDriveTargetVelocity", (y + x)*goal);
    SmartDashboard.putNumber("RightDriveTargetVelocity", (y - x)*goal); 
  }

  //Arm wheels on gripper
  public void shooter(){
    if(timer.get() <= shooterFlag){
      armWheels.set(ControlMode.PercentOutput, -power); 
    }else{
      armWheels.set(ControlMode.PercentOutput, 0);
    }
  }

  //safety
  public void liftSafetyShutoff(double timeToKill, double ampToKill){
    double amps = Math.abs(liftMotor.getSupplyCurrent());
    if (amps > ampToKill && !liftSafetyTriggered){
      liftSafetyTriggered = true;
      safetyTimerLift = timer.get() + timeToKill;
    }

    if (amps < ampToKill){
      liftSafetyTriggered = false;
    }

    if (liftSafetyTriggered && timer.get() >= safetyTimerLift){
      //this means we should shut off
      liftAllowedToRun = false;
      manualMode = true;
    }
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}