package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class ParallelCommandGroup extends Command {

    private List<Command> commands = new ArrayList<>();

    public ParallelCommandGroup(Command... commands) {addCommands(commands);}

    public final void addCommands(Command... commands) {
        for (Command command : commands) {
            for (Subsystem requirement : command.getRequirements()) {
                if (getRequirements().contains(requirement)) {
                    double parallelCommandGroupErrorSameSubsystemInUse = 10/0;    //TODO Remove before comp!!!!
                }
            }
            this.commands.add(command);
            addRequirements(command.getRequirements());
        }
    }

    @Override
    public void init() {
        if (!commands.isEmpty()) {
            for (Command command : commands) {
                command.init();
            }
        }
    }

    @Override
    public void run() {
        if (commands.isEmpty()) return;

        Iterator<Command> iterator = commands.iterator();
        while (iterator.hasNext()) {
            Command command = iterator.next();
            command.run();
            if (command.isFinished()) {
                command.stop(false);
                iterator.remove();
            }
        }

    }

    @Override
    public void stop(boolean isInterrupted) {
        if (commands.isEmpty()) return;

        for (Command command : commands) {
            command.stop(isInterrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return commands.isEmpty();
    }

}
