package frc.team6220.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team6220.robot.Robot;

public class Cruise extends Command {

    private static Timer time = new Timer();
    private boolean isFinished;
    private double cruisetime;
    private double distance;

    public Cruise(double distance){
        requires(Robot.DriveTrain);
        this.distance = distance / 9.583;
    }

    protected void initialize(){
        time.reset();
        time.start();
        cruisetime = getCruiseTime(SpeedUp.TimeTook, distance, Robot.DriveTrain.max_vel, Robot.DriveTrain.a);
    }

    protected void execute(){
        double leftspeed = Robot.DriveTrain.v + Robot.DriveTrain.compensationFactor();
        double rightspeed = Robot.DriveTrain.v - Robot.DriveTrain.compensationFactor();
        Robot.DriveTrain.set(ControlMode.PercentOutput, leftspeed, rightspeed);
        isFinished = time.get() >= cruisetime;
    }

    protected boolean isFinished() {
        return isFinished;
    }

    protected void end(){
        time.stop();
    }

    protected void interrupted(){
        end();
    }

    public double getCruiseTime(double time, double distance, double max_vel, double max_accel) {
        double x = 0.5 * max_accel * Math.pow(time, 2);
        double cruise_dist = Math.abs(distance) - (2 * x);
        double cruise_time = cruise_dist / max_vel;
        return cruise_time;
    }
}
