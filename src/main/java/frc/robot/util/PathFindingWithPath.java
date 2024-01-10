package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;

public class PathFindingWithPath {
    public static Command pathFindingAutoBuilder(String endPath, JoystickButton button) {
        PathPlannerPath endPathTraj = PathPlannerPath.fromPathFile(endPath);

        PathConstraints constraints = new PathConstraints(
            3, 4, 
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

        Command pathFindingCommand = AutoBuilder.pathfindThenFollowPath(endPathTraj, constraints, 0);
        return pathFindingCommand.until(() -> !(button.getAsBoolean()));
    } 
}
