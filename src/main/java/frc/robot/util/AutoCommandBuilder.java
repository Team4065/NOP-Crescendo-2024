package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommandBuilder {
    public static Command returnAutoCommand(String autoPathName) {
        Command autoCmd = AutoBuilder.buildAuto(autoPathName);

        return autoCmd;
    }
}
