package frc.team6220.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team6220.robot.Robot;

public class SpeedUp extends Command {

    public static double TimeTook;
    private boolean isFinished = false;

    private static Timer time = new Timer();

    public SpeedUp(){
        requires(Robot.DriveTrain);
    }

    protected void initialize(){
        Robot.DriveTrain.gyro.reset();
        time.reset();
        time.start();
    }

    protected void execute(){
        Robot.DriveTrain.v += Robot.DriveTrain.a;
        double leftspeed = Robot.DriveTrain.v + Robot.DriveTrain.compensationFactor();
        double rightspeed = Robot.DriveTrain.v - Robot.DriveTrain.compensationFactor();
        Robot.DriveTrain.set(ControlMode.PercentOutput, leftspeed, rightspeed);
        isFinished = Robot.DriveTrain.v >= Robot.DriveTrain.max_vel;

    }

    protected boolean isFinished() {
        return isFinished;
    }

    protected void end(){
        TimeTook = time.get();
        time.stop();
        Robot.DriveTrain.v = 0.5;
    }

    protected void interrupted(){
        end();
    }
}
