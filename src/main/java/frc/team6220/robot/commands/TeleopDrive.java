package frc.team6220.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team6220.robot.OI;
import frc.team6220.robot.Robot;

import static frc.team6220.robot.Robot.DriveTrain;

public class TeleopDrive extends Command {

    public TeleopDrive(){
        requires(Robot.DriveTrain);
        requires(Robot.Elevator);
        requires(Robot.Intake);
    }

    protected void initialize(){

    }

    protected void execute(){
        Robot.DriveTrain.teleopDrive(Robot.oi.getLeftJoystick(), Robot.oi.getRightJoystick());
        Robot.Elevator.teleopDrive(Robot.oi.getXboxController());
        Robot.Intake.teleopDrive(Robot.oi.getXboxController());
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end(){
        Robot.DriveTrain.stop();
    }

    protected void interrupted(){
        end();
    }
}
