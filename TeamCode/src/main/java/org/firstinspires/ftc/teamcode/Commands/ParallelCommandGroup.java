package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

public class ParallelCommandGroup extends Command {

    private HashMap<Command, Boolean> commands = new HashMap<>();

    public ParallelCommandGroup(Command... commands) {addCommands(commands);}

    public final void addCommands(Command... commands) {
        for (Command command : commands) {
            for (Subsystem requirement : command.getRequirements()) {
                if (getRequirements().contains(requirement)) {
                    double parallelCommandGroupErrorSameSubsystemInUse = 10/0;    //TODO Remove before comp!!!!
                }
            }
            this.commands.put(command, false);
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

        for (Command command : commands.keySet()) {
            if (commands.get(command)) {
                continue;
            }
            command.run();
            if (command.isFinished()) {
                command.stop(false);
                commands.put(command, true);
            }
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
