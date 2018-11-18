package frc.team6220.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveTo extends CommandGroup {

    public DriveTo(double distance){
        addSequential(new SpeedUp());
        addSequential(new Cruise(distance));
        addSequential(new SlowDown());
    }

}
