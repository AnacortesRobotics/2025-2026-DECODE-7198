package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

public class ParallelRaceCommandGroup extends Command {

    private HashMap<Command, Boolean> commands = new HashMap<>();

    public ParallelRaceCommandGroup(Command... commands) {addCommands(commands);}

    public final void addCommands(Command... commands) {
        for (Command command : commands) {
            for (Subsystem requirement : command.getRequirements()) {
                if (getRequirements().contains(requirement)) {
                    double parallelCommandGroupErrorSameSubsystemInUse = 10/0;    //TODO Remove before comp!!!!
                }
            }
            this.commands.put(command, true);
            addRequirements(command.getRequirements());
        }
    }

    @Override
    public void init() {
        if (!commands.isEmpty()) {
            for (Command command : commands.keySet()) {
                command.init();
                commands.put(command, false);
            }
        }
    }

    @Override
    public void run() {
        if (commands.isEmpty()) return;

        boolean isDone = false;

        for (Command command : commands.keySet()) {
            command.run();
            if (command.isFinished()) {
                command.stop(false);
                isDone = true;
                break;
            }
        }
        if (isDone) {
            stop(false);
            commands.clear();
        }

    }

    @Override
    public void stop(boolean isInterrupted) {
        if (commands.isEmpty()) return;

        for (Command command : commands.keySet()) {
            command.stop(isInterrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return commands.isEmpty();
    }

}
