package org.firstinspires.ftc.teamcode.Commands;

import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

/**
 * A state machine representing a complete action to be performed by the robot.
 *
 * <p>Commands are the building blocks of robot behavior. They can be combined into
 * command groups to create complex autonomous routines or tele-op behaviors.
 */
public abstract class Command {

    /** The set of subsystems required by this command. */
    private final Set<Subsystem> requirements = new HashSet<>();

    /** Whether the command can be interrupted by another command that shares a requirement. */
    private boolean interruptable = false;

    /** The name of the command. */
    private String name = "unnamed";

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    public void init() {}

    /**
     * The main body of a command.
     */
    public void run() {}

    /**
     * The action to take when the command ends. Called when either {@link #isFinished()}
     * returns true or the command is interrupted.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    public void stop(boolean interrupted) {}

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will
     * call its stop() method and unschedule it.
     *
     * @return whether the command has finished
     */
    public boolean isFinished() {return false;}

    /**
     * Adds the specified subsystems to the requirements of the command.
     * The scheduler will now prevent other commands from using these subsystems
     * while this command is running.
     *
     * @param subsystems the subsystems to add
     * @return the command itself for chaining
     */
    public Command addRequirements(Subsystem... subsystems) {
        requirements.addAll(Arrays.asList(subsystems));
        return this;
    }

    /**
     * Adds the specified collection of subsystems to the requirements of the command.
     *
     * @param subsystems the subsystems to add
     * @return the command itself for chaining
     */
    public Command addRequirements(Collection<Subsystem> subsystems) {
        requirements.addAll(subsystems);
        return this;
    }

    /**
     * Specifies the set of subsystems used by this command.
     *
     * @return the set of subsystems required by this command
     */
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    /**
     * Decorates this command with a companion that will run after this command finishes.
     *
     * @param command the command to run next
     * @return the decorated command
     */
    public SequentialCommandGroup andThen(Command command) {
        return new SequentialCommandGroup(this, command);
    }

    /**
     * Decorates this command with a runnable that will run after this command finishes.
     *
     * @param toRun the runnable to run next
     * @param requirements the subsystems required by the runnable
     * @return the decorated command
     */
    public SequentialCommandGroup andThen(Runnable toRun, Subsystem... requirements) {
        return andThen(new InstantCommand(toRun, requirements));
    }

    /**
     * Sets whether the command is interruptible.
     *
     * @param interruptable whether the command is interruptible
     * @return the command itself for chaining
     */
    public Command setInterruptable(boolean interruptable) {
        this.interruptable = interruptable;
        return this;
    }

    /**
     * Whether the command is interruptible.
     *
     * @return <b>true</b> if the command is interruptible, <b>false</b> if not.
     */
    public boolean isInterruptable() {
        return interruptable;
    }

    /**
     * Sets the name of the command.
     *
     * @param name the name of the command
     * @return the command itself for chaining
     */
    public Command setName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Returns the name of the command.
     *
     * @return the name of the command
     */
    public String getName() {
        return name;
    }

    /**
     * Schedules a task to cancel this command via the CommandScheduler.
     *
     * @return an InstantCommand that cancels this command
     */
    public Command cancel() {
        return new InstantCommand(()->{CommandScheduler.getInstance().cancelCommand(this);});
    }

}
