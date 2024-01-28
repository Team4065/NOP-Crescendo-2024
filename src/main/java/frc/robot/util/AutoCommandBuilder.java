package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AutoCommandBuilder {
    public static Command returnAutoCommand(String autoPathName) {
        Command autoCmd = AutoBuilder.buildAuto(autoPathName);
        Constants.autoRoutines.put(autoCmd, autoPathName);
        
        return autoCmd;
    }
}
