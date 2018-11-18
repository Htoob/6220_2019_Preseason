package frc.team6220.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team6220.robot.Robot;

public class SlowDown extends Command {

    private boolean isFinished;
    public SlowDown(){
        requires(Robot.DriveTrain);
    }

    protected void initialize(){

    }

    protected void execute(){
        Robot.DriveTrain.v -= Robot.DriveTrain.a;
        double leftspeed = Robot.DriveTrain.v + Robot.DriveTrain.compensationFactor();
        double rightspeed = Robot.DriveTrain.v - Robot.DriveTrain.compensationFactor();
        Robot.DriveTrain.set(ControlMode.PercentOutput, leftspeed, rightspeed);
        isFinished = Robot.DriveTrain.v <= 0;
    }

    protected boolean isFinished() {
        return isFinished;
    }

    protected void end(){
        Robot.DriveTrain.v = 0;
        Robot.DriveTrain.stop();
    }

    protected void interrupted(){
        end();
    }
}
