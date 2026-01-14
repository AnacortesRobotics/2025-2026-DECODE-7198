package org.firstinspires.ftc.teamcode.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * A command that allows the user to pass in functions for each of the basic command methods.
 * Useful for inline definitions of complex functionality without extending {@link Command}
 * in a separate file.
 */
public class FunctionalCommand extends Command {

    /** The function to run when the command is initialized. */
    private final Runnable onInit;
    /** The function to run when the command is scheduled. */
    private final Runnable onRun;
    /** The function to run when the command ends. */
    private final Consumer <Boolean> onStop;
    /** The state of the command - finished or not finished. */
    private final BooleanSupplier isFinished;

    /**
     * Creates a new FunctionalCommand.
     *
     * @param onInit the function to run at command initialization
     * @param onRun the function to run on command execution
     * @param onStop the function to run on command end
     * @param isFinished the function that indicates whether the command has finished
     * @param requirements the subsystems required by this command
     */
    public FunctionalCommand(Runnable onInit,
                             Runnable onRun,
                             Consumer <Boolean> onStop,
                             BooleanSupplier isFinished,
                             Subsystem... requirements)
    {
        this.onInit = onInit;
        this.onRun = onRun;
        this.onStop = onStop;
        this.isFinished = isFinished;
        addRequirements(requirements);
    }

    /**
     * Creates a new FunctionalCommand by copying the functions from another FunctionalCommand.
     *
     * @param command the FunctionalCommand to copy functions and requirements from
     */
    public FunctionalCommand(FunctionalCommand command) {
        onInit = command.onInit;
        onRun = command.onRun;
        onStop = command.onStop;
        isFinished = command.isFinished;
        addRequirements(command.getRequirements());
    }

    /**
     * Run the initial subroutine of a command.
     * Called once when the command is initially scheduled.
     */
    @Override
    public void init() {
        onInit.run();
    }

    /**
     * Runs the main body of a command (passed in constructor).
     */
    @Override
    public void run() {
        onRun.run();
    }


    /**
     * Runs the action to take when the command ends (passed in constructor).
     * Called when either the command finishes normally, or when it is interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void stop(boolean interrupted) {
        onStop.accept(interrupted);
    }

    /**
     * Returns whether the command has finished.
     *
     * @return <b>true</b> if the command has finished,
     * <b>false</b> if command has not finished.
     */
    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }


}
