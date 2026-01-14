package org.firstinspires.ftc.teamcode.Commands;

/**
 * A command that runs another command repeatedly.
 *
 * <p>The command is restarted every time it finishes, until the specified number
 * of loops is reached. If no loop count is specified, it repeats indefinitely.
 */
public class RepeatCommand extends Command {

    /** The command to be repeated. */
    private Command repeatCommand;

    /** The maximum number of times the command should run (-1 for infinite). */
    private int maxLoops;

    /** The number of times the command has completed its execution. */
    private int currentLoops = 0;

    /** Whether the wrapped command is currently in a stopped state between loops. */
    private boolean isStopped = false;

    /**
     * Creates a new RepeatCommand that repeats the given command indefinitely.
     *
     * @param command the command to repeat
     */
    public RepeatCommand(Command command) {
        this(command, -1);
    }

    /**
     * Creates a new RepeatCommand that repeats the given command a set number of times.
     *
     * @param command the command to repeat
     * @param loops the number of times to repeat the command (-1 for infinite)
     */
    public RepeatCommand(Command command, int loops) {
        repeatCommand = command;
        addRequirements(repeatCommand.getRequirements());
        maxLoops = loops;
    }

    /**
     * Initializes the wrapped command and resets the loop counter.
     */
    @Override
    public void init() {
        repeatCommand.init();
        currentLoops = 0;
    }

    /**
     * Checks if the wrapped command has finished and restarts it if necessary.
     */
    @Override
    public void run() {
        if (currentLoops == maxLoops) return;
        if (isStopped && (currentLoops < maxLoops || maxLoops == -1)) {
            repeatCommand.init();
            isStopped = false;
        }
        repeatCommand.run();
        if (repeatCommand.isFinished()) {
            repeatCommand.stop(false);
            isStopped = true;
            if (maxLoops != -1) {currentLoops += 1;}
        }
    }

    /**
     * Stops the wrapped command.
     *
     * @param isInterrupted whether the command was interrupted/canceled
     */
    @Override
    public void stop(boolean isInterrupted) {
        if (!isStopped) {
            repeatCommand.stop(true);
        isStopped = true;
        }
    }

    /**
     * Whether the command has finished repeating.
     *
     * @return <b>true</b> if the command has run the specified number of loops
     */
    @Override
    public boolean isFinished() {
        return maxLoops != -1 && currentLoops == maxLoops;
    }

}
