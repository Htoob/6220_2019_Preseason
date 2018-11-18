package frc.team6220.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team6220.robot.ADIS16448_IMU;
import frc.team6220.robot.RobotMap;
import frc.team6220.robot.commands.TeleopDrive;

public class DriveTrain extends Subsystem implements PIDOutput {

    private VictorSPX Ldrive1 = new VictorSPX(RobotMap.Ldrive1);
    private VictorSPX Ldrive2 = new VictorSPX(RobotMap.Ldrive2);
    private VictorSPX Rdrive1 = new VictorSPX(RobotMap.Rdrive1);
    private VictorSPX Rdrive2 = new VictorSPX(RobotMap.Rdrive2);

    public ADIS16448_IMU gyro = new ADIS16448_IMU(ADIS16448_IMU.Axis.kZ, ADIS16448_IMU.AHRSAlgorithm.kMadgwick);

    public final PIDController pid;

    //PID controller constants
    private final double kP = 1;
    private final double kI = 1;
    private final double kD = 1;

    //Manual PID constants
    private final double Kp = 1.0/360;
    private final double Ki = 1.0/3600;
    private final double Kd = 0;

    public static double max_vel = 0.5;
    public static double v = 0;
    public static double a = 0.02;

    public double error = 0;
    public double compoundError = 0;

    public DriveTrain(){
        Ldrive1.setNeutralMode(NeutralMode.Brake);
        Ldrive2.setNeutralMode(NeutralMode.Brake);
        Rdrive1.setNeutralMode(NeutralMode.Brake);
        Rdrive2.setNeutralMode(NeutralMode.Brake);

        gyro.calibrate();

        pid = new PIDController(kP, kI, kD, gyro, this);
        pid.setInputRange(-180, 180);
        pid.setOutputRange(-0.5, 0.5);
        pid.setAbsoluteTolerance(0.5);
        pid.setContinuous();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new TeleopDrive());
    }

    public double getAngle(){
        double robotangle = gyro.getAngleZ()%360;
        if(robotangle<-185){
            robotangle += 360;
        }
        else if(robotangle > 185){
            robotangle -=360;
        }
        return robotangle;
    }

    public void pidWrite(double output){
        set(ControlMode.PercentOutput, output, -output);
    }

    public void rotateDegreesPID(double angle){
        gyro.reset();
        pid.reset();
        pid.setPID(kP, kI, kD);
        pid.setSetpoint(angle);
        pid.enable();
    }

    public void rotateDegrees(double angle){
        double prevError = error;
        error = angle - getAngle();
        compoundError += error;
        double deltaError = error - prevError;

        double output = (Kp*error) + (Ki*compoundError) + (Kd*deltaError);
        set(ControlMode.PercentOutput, output, output);
    }

    public double compensationFactor(){
        if(Math.abs(error) > 0.5) {
            double prevError = error;
            error = 0 - getAngle();
            compoundError += error;
            double deltaError = error - prevError;

            return (Kp * error) + (Ki * compoundError) + (Kd * deltaError);
        }
        else{
            compoundError = 0;
            return 0;
        }
    }

    public void teleopDrive(Joystick js1, Joystick js2){
        if(js1.getTrigger()) {
            Ldrive1.set(ControlMode.PercentOutput, js1.getY() * -((js1.getThrottle() - 1) / 2));
            Ldrive2.set(ControlMode.PercentOutput, js1.getY() * -((js1.getThrottle() - 1) / 2));
        }
        if(js2.getTrigger()) {
            Rdrive1.set(ControlMode.PercentOutput, js2.getY() * -((js1.getThrottle() - 1) / 2));
            Rdrive2.set(ControlMode.PercentOutput, js2.getY() * -((js1.getThrottle() - 1) / 2));
        }
    }

    public void set(ControlMode mode, double leftSpeed, double rightSpeed){
        Ldrive1.set(mode, leftSpeed);
        Ldrive2.set(mode, leftSpeed);
        Rdrive1.set(mode, -rightSpeed);
        Rdrive2.set(mode, -rightSpeed);
    }

    public void stop(){
        set(ControlMode.PercentOutput, 0 ,0);
    }
}
