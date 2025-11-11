package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class SequentialCommandGroup extends Command {

    private List<Command> commands = new ArrayList<>();

    public int commandIndex = -1;

    public SequentialCommandGroup(Command... commands) {
        addCommands(commands);
    }

    public final void addCommands(Command... commands) {
        for (Command command : commands) {
            this.commands.add(command);
            addRequirements(command.getRequirements());
        }
    }

    @Override
    public void init() {
        commandIndex = 0;
        if (!commands.isEmpty()) {
            commands.get(0).init();
        }
    }

    @Override
    public void run() {
        if (commands.isEmpty()) {return;}

        Command currentCommand = commands.get(commandIndex);

        currentCommand.run();

        if (currentCommand.isFinished()) {
            currentCommand.stop(false);
            commandIndex += 1;

            if (commandIndex < commands.size()) {
                commands.get(commandIndex).init();
            }
        }
    }

    @Override
    public void stop(boolean isInterrupted) {
        if (commands.isEmpty() || commandIndex >= commands.size() || commandIndex < 0) {return;}

        if (isInterrupted) {
            commands.get(commandIndex).stop(true);
        }
        commandIndex = -1;
    }

    public boolean isFinished() {
        return commandIndex >= commands.size();
    }

}
