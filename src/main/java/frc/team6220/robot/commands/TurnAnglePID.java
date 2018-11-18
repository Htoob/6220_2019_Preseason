package frc.team6220.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team6220.robot.Robot;

public class TurnAnglePID extends Command {

    double Angle;

    boolean isFinished = false;
    boolean inErrorZone = false;
    int count = 0;

    public TurnAnglePID(double angle){
        requires(Robot.DriveTrain);
        Angle = angle;
    }

    protected void initialize(){
        Robot.DriveTrain.rotateDegreesPID(Angle);
    }

    protected void execute(){
        double error = Robot.DriveTrain.pid.getError();
        inErrorZone = Math.abs(error) < 0.5;
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
        Robot.DriveTrain.pid.disable();
        Robot.DriveTrain.stop();
    }

    protected void interrupted() {
        end();
    }
}
