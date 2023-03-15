package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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
import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {


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
  int autoType = 1;
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

  double[] armRotatePositions = {0, 226174, 336000, 370000, 402000, 37149, 362000, -76753};
  double[] armLiftPositions = {0, -531184, -531000, -358000, -358000, 0, -375000, -640000};
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

  //Collection<TalonFX> _fxes =  { new TalonFX(1), new TalonFX(2), new TalonFX(3) };

  //auto Variables
  int autoStep = 0;

  @Override
  public void robotInit() {
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

    //testMotor = new TalonSRX(24); //this was on the comp bot for the rotating arm
    armMotor = new TalonSRX(11);
    armMotor.setInverted(true);
    armMotor.setSensorPhase(false);
    armMotor.config_kP(0, 0.5);
    armMotor.configClosedloopRamp(0.2);
    //armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 12, ))

    armRotator = new TalonFX(50);
    armRotator.setInverted(false);

    armWheels = new TalonSRX(24);
    armWheels.setInverted(true); //NEEDS TO BE CHANGED ON PRACTICE BOT

    rightDrive1.setInverted(true);
    rightDrive2.setInverted(true);

    leftDrive2.set(ControlMode.Follower, 1);
    rightDrive2.set(ControlMode.Follower, 3);

    leftDrive1.setNeutralMode(NeutralMode.Coast);
    leftDrive2.setNeutralMode(NeutralMode.Coast);
    rightDrive1.setNeutralMode(NeutralMode.Coast);
    rightDrive2.setNeutralMode(NeutralMode.Coast);

    leftDrive1.setSensorPhase(true);
    rightDrive1.setSensorPhase(true);

    leftDrive1.config_kP(0, 0.01);
    leftDrive1.config_kI(0, 0.0003);
    leftDrive1.config_kD(0, 1);

    rightDrive1.config_kP(0, 0.01);
    rightDrive1.config_kI(0, 0.0003);
    rightDrive1.config_kD(0, 1);



    leftDrive1.configClosedLoopPeakOutput(0, 1);
    rightDrive1.configClosedLoopPeakOutput(0, 1);

    driveStick = new Joystick(0);
    operatorStick = new Joystick(1);

    navX = new AHRS();

    navX.calibrate();

    isSticking = false;

    brakeStatus = true;

    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    driveGearShiftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    SmartDashboard.putNumber("angleP", angleP);
    SmartDashboard.putNumber("angleD", angleD);

    timer = new Timer();
    timer.start();
    //lft PID
    liftP = 0.02;
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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //SmartDashboard.putNumber("shooter power (0-1)", 1);
    //SmartDashboard.putNumber("shooter runtime", 2); :)
    power = SmartDashboard.getNumber("shooter power (0-1)", 1);
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
    SmartDashboard.putNumber("Lift Current", liftMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Lift MAX Current", highestAmperage);

    //SmartDashboard.putNumber("ArmP", 0);

    SmartDashboard.putBoolean("Lift Allowed ON:", liftAllowedToRun);

    SmartDashboard.putBoolean("High Gear", driveShiftBool);

    liftSafetyShutoff(0.1, 10);

    if (liftMotor.getSupplyCurrent() > highestAmperage){
      highestAmperage = liftMotor.getSupplyCurrent();
    }

    if (!liftAllowedToRun){
      liftMotor.set(ControlMode.Disabled, 0);
    }

    SmartDashboard.putNumber("Auto Step", autoStep);

    SmartDashboard.putNumber("LeftDrive POS", leftDrive1.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightDrive POS", rightDrive1.getSelectedSensorPosition());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
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
      case 0:
        auto1_DropPieceMoveBackwardNoBalance();
        break;
        case 1:
          auto2_DropPieceMoveBackwardBalanceBackward();
          break;
        case 2:
          auto2_DropPieceMoveBackwardBalanceBackward();
          break;
    }
    
  }

  public void auto1_DropPieceMoveBackwardNoBalance(){
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
        armMotor.set(ControlMode.Position, armSlidePositions[2]);
        //armPID(armSlidePositions[2]);
        armRotator.set(ControlMode.Position, armRotatePositions[2]);
        if (liftAllowedToRun) {liftMotor.set(ControlMode.Position, armLiftPositions[2]);}
        autoStep++;
        break;
      case 2:
        if (armRotator.getSelectedSensorPosition() < armRotatePositions[2] * 0.9){
          autoStep++;
          autoTimer = timer.get();
        }
        break;
      case 3:
        if (timer.get() - autoTimer > 2.5){
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
        armMotor.set(ControlMode.Position, armSlidePositions[7]);
        armRotator.set(ControlMode.Position, armRotatePositions[7]);
        if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[7]);
        leftDrive1.set(ControlMode.Velocity, 3000);
        rightDrive1.set(ControlMode.Velocity, 3000);
        break;
    }
  }

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
        armMotor.set(ControlMode.Position, armSlidePositions[7]);
        armRotator.set(ControlMode.Position, armRotatePositions[7]);
        if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[7]);
        leftDrive1.set(ControlMode.Velocity, 7000);
        rightDrive1.set(ControlMode.Velocity, 7000);
        autoStep++;
        break;
      case 6:
        if (navX.getPitch() < -8){
          //activate balcncing code
          autoStep++;
        }
        break;
      case 7:
        setBrakeMode(true);
        balanceRobot_DrivingBackward();
        break;
    }
  }

  public void autoDriveToPositionVelocityDrive(int distanceGoal, int goalVelocity, int maxOut, double p){
    double error = distanceGoal - ((leftDrive1.getSelectedSensorPosition() + rightDrive1.getSelectedSensorPosition()) / 2);
    double output = error * goalVelocity * p;

    output = output > maxOut ? maxOut : output;
    output = output < -maxOut ? -maxOut : output;

    leftDrive1.set(ControlMode.Velocity, output);
    rightDrive1.set(ControlMode.Velocity, output);
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

    liftAllowedToRun = SmartDashboard.getBoolean("Lift Allowed ON:", false);

    //armMotor.setSelectedSensorPosition(0);

    armRotator.config_kP(0, 0.01);
    armRotator.config_kI(0, 0.0);
    armRotator.config_kD(0, 0.0);

    leftDrive1.setSelectedSensorPosition(0);
    rightDrive1.setSelectedSensorPosition(0);


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

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
      if (driveStick.getRawButton(14)){
        //balanceRobot_DrivingBackward();
        //setBrakeMode(true);
        driveShiftBool = false;
        autoDriveToPositionVelocityDrive(300000, 5000, 5000, 0.0001);
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

    //if(driveStick.getRawButtonPressed(10)){
      //fastDriving = !fastDriving;
    //}
    ////////////////////////
    //pneumatics 
    /////////////

    
    if(driveStick.getRawButtonPressed(10)){
      driveShiftBool = !driveShiftBool;
    }
    driveGearShiftSolenoid.set(driveShiftBool);


    if(operatorStick.getRawButtonPressed(4)){
      gripperBool = !gripperBool;
    }
    gripperSolenoid.set(gripperBool);
     
    // if(driveStick.getRawButtonPressed(3)){
    //     if(compState == false){
    //      compressor.disable();
    //      }
    //      else if(compState == true){
    //        compressor.enableDigital();
    //      } 
    //      compState = !compState;  
    // }
    if(driveStick.getRawButtonPressed(3)){
      shooterFlag = timer.get() + time;
    }
    if(driveStick.getRawButton(3)){ //out
      shooter();
      // armWheels.set(ControlMode.PercentOutput, -1);
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
    //}if(operatorStick.getRawButton(8)){
    //   liftPID(50); //keep in percent form, not decimal form 
    // }else if(operatorStick.getRawButton(7)){
    //   liftPID(0);
    //}else{
    //}


     //////////////////
     //end of pneumatics */
     //////////////////

    /*if(driveStick.getRawButtonPressed(5) && driveStick.getRawButtonPressed(6)){
      brakeStatus = !brakeStatus;
      setBrakeMode();
    }*/

    //lift
    /*
    if(driveStick.getRawButton(1)) {
      liftMotor.set(ControlMode.Position, (int)(25000)); //pid low
    } 
    else if(driveStick.getRawButton(2)) {
      liftMotor.set(ControlMode.Position, (int)(105000));//0.65*maxPosition)); //pid high
    } 
    else{
      liftMotor.set(ControlMode.PercentOutput, 0);
      }*/
      /*
    if(driveStick.getRawButtonPressed(9) || driveStick.getRawButtonPressed(10)){
      //set the flags for the amp code
      zeroCompleted = false;
    }*/

    if (operatorStick.getRawButtonReleased(10)){
      armMotor.setSelectedSensorPosition(0);
      armRotator.setSelectedSensorPosition(0);
      liftMotor.setSelectedSensorPosition(0);
      armAutoPosition = 0;
    }

    if (manualMode){
      
      /*if (operatorStick.getRawButton(6)){
        armSlideGoal = 9700;
      }
      else if (operatorStick.getRawButton(8)){
        armSlideGoal = 19000;
      }
      else if (operatorStick.getRawButton(5)){
        armSlideGoal = 0;
      }*/
      if (operatorStick.getRawButton(6)){
        armMotor.set(ControlMode.PercentOutput, 0.4);
      }
      else if (operatorStick.getRawButton(5)){
        armMotor.set(ControlMode.PercentOutput, -0.4);
      }
      else{
        //armPID(armSlideGoal);
        armMotor.set(ControlMode.PercentOutput, 0);
      }
      
      

      if (liftAllowedToRun) liftMotor.set(ControlMode.PercentOutput, -operatorStick.getRawAxis(3));

      armRotator.set(ControlMode.PercentOutput, -Math.pow(operatorStick.getRawAxis(1), 3));
      //auto arm stuff


    }
    else{ //not in manual mode


      //arm rotation
      
      if(operatorStick.getRawButton(5)){
        armAutoPosition = 1;
      }
      if(armAutoPosition == 1) {
        if(operatorStick.getRawButton(1)){
          armAutoPosition = 0;
        }else if(operatorStick.getRawButton(6)){
          armAutoPosition = 2;
        }
        else if(operatorStick.getRawButton(7)){
          armAutoPosition = 3;
        }
        else if(operatorStick.getRawButton(8)){
          armAutoPosition = 4;
        }
        else if(operatorStick.getPOV() == 180){
          armAutoPosition = 5;
        } else if (operatorStick.getRawButton(3)) {
          armAutoPosition = 6;
        } else if (operatorStick.getRawButton(2)) {
          armAutoPosition = 7;
        }
      }

      //armMotor.set(ControlMode.Position, armSlidePositions[armAutoPosition]);
      //double armSlideGoalFixed = 
      //armPID(armSlidePositions[armAutoPosition]);
      armMotor.set(ControlMode.Position, armSlidePositions[armAutoPosition]);
      if (liftAllowedToRun) liftMotor.set(ControlMode.Position, armLiftPositions[armAutoPosition]);
      armRotator.set(ControlMode.Position, armRotatePositions[armAutoPosition]);

////////////////////////////////////////////////////////////////////////////////////////////////// the wall

    }
  }

  public void balanceRobot_DrivingBackward(){
    double curPitch = Math.sin(Math.toRadians(navX.getPitch()));
    double p = 4;

    double output = -curPitch * p;

    int maxOut = 2500;

    output = output * maxOut;

    output = output > maxOut ? maxOut : output;
    output = output < -maxOut ? -maxOut : output;

    leftDrive1.set(ControlMode.Velocity, output);
    rightDrive1.set(ControlMode.Velocity, output);
  }

  //FIXME: use internal motor PID
  public void armPID(double Goal){
    double error = Goal - armMotor.getSelectedSensorPosition();
    double p = 0.00025;
    double output = error*p;

    double max = 1;
    output = output > max ? max: output;
    output = output < -max ? -max: output;

    armMotor.set(ControlMode.PercentOutput, output);
  }

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
  public void armRotatePID(double percentGoal){
    double goal = armMaxPosition * percentGoal/100;
    
    armRotator.set(ControlMode.Position, goal);
  }


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

  public void setBrakeMode(boolean setMe){
    if (setMe == true){
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

  public void balancingDistancePID(double goal, double P, double D){
    double position = (leftDrive1.getSelectedSensorPosition() + rightDrive1.getSelectedSensorPosition()) / 2;
    double error = goal - position;


    if(error <= goal - 1000){
      leftDrive1.set(ControlMode.PercentOutput, 0);
      leftDrive2.set(ControlMode.PercentOutput, 0);
      rightDrive1.set(ControlMode.PercentOutput, 0);
      rightDrive2.set(ControlMode.PercentOutput, 0);
    }
  }

    
   public void balancingPID(double P, double D, double max){
    double error = navX.getRoll();
    double demand = error * P;

    demand = (demand > max)? max:demand;
    demand = (demand < -max)? -max:demand;

    leftDrive1.set(ControlMode.PercentOutput, demand);
    rightDrive1.set(ControlMode.PercentOutput, demand);
  }

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

  public void driveButBetter(){
    double x;
    double y;
    double maxY = 1;
    //max goal = -20000
    double goal = 20000;

    //Negative X for 2023 practice bot, positive X for 2022 comp bot
    x = -driveStick.getRawAxis(0) / 3;
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

    if(x == 0 && y == 0) {
      leftDrive1.set(ControlMode.Disabled, 0);
      rightDrive1.set(ControlMode.Disabled, 0);
    } else {
      leftDrive1.set(ControlMode.Velocity, (y + x)*goal );
      rightDrive1.set(ControlMode.Velocity, (y - x)*goal);
    }

    SmartDashboard.putNumber("LeftDriveTargetVelocity", (y + x)*goal);
    SmartDashboard.putNumber("RightDriveTargetVelocity", (y - x)*goal);

    
  }
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
    }
  }
}