package frc.team6220.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team6220.robot.Robot;

public class TurnAngle extends Command {

    double Angle;

    boolean isFinished = false;
    boolean inErrorZone = false;

    double count = 0;

    public TurnAngle(double angle){
        requires(Robot.DriveTrain);
        Angle = angle;
    }

    protected void initialize(){
        Robot.DriveTrain.gyro.reset();
        Robot.DriveTrain.error = 0;
        Robot.DriveTrain.compoundError = 0;
    }

    protected void execute(){
        Robot.DriveTrain.rotateDegrees(Angle);
        inErrorZone = Math.abs(Robot.DriveTrain.error) < 0.5;
        if(inErrorZone){
            count++;
            isFinished = count >= 5;
        }
        else{
            count = 0;
        }
    }

    protected boolean isFinished() {
        return isFinished;
    }

    protected void end(){
        Robot.DriveTrain.stop();
    }

    protected void interrupted() {
        end();
    }
}
