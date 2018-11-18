package frc.team6220.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoCommands extends CommandGroup {

    public AutoCommands(){
        //addSequential(new DriveTo(10));
        //addSequential(new TurnAnglePID(180));
        //addSequential(new DriveTo(10));
        addSequential(new TurnAngle(90));
    }

}
