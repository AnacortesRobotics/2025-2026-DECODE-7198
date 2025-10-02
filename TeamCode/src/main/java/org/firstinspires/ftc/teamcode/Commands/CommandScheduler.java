package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class CommandScheduler {

    private static CommandScheduler instance;

    private Telemetry telemetry;

    private Set<Command> scheduledCommands = new LinkedHashSet<>();
    private LinkedHashMap<Subsystem, Command> activeSubsystems = new LinkedHashMap<>();
    private Set<Command> activeCommands = new LinkedHashSet<>();

    public static synchronized CommandScheduler getInstance() {
        if (instance == null) {instance = new CommandScheduler();}
        return instance;
    }

    public void init(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void schedule(Command... commands) {
        scheduledCommands.addAll(Arrays.asList(commands));

    }

    public void run() {
        if (scheduledCommands.isEmpty() && activeSubsystems.isEmpty()) {return;}

        Iterator<Command> commandIterator = scheduledCommands.iterator();
        while (commandIterator.hasNext()) {
            Command command = commandIterator.next();
            Set<Command> conflicts = getActiveConflicts(command);
            Set<Subsystem> requirements = command.getRequirements();
            boolean conflicting = false;
            boolean willInterrupt = false;
            for (Command conflict : conflicts) {
                if (conflict.isInterruptable()) {
                    willInterrupt = true;
                } else {
                    conflicting = true;
                    break;
                }
            }

            if (!conflicting) {
                if (willInterrupt) {
                    for (Command conflict : conflicts) {
                        endCommand(conflict, true);
                    }
                }
                for (Subsystem requirement : requirements) {
                    activeSubsystems.put(requirement, command);
                }
                commandIterator.remove();
                activeCommands.add(command);
                command.init();
            }
        }

        for (Command command : activeCommands) {
            command.run();
            if (command.isFinished()) {
                endCommand(command, false);
            }
        }

    }



    private Set<Command> getActiveConflicts(Command command) {
        Set<Command> conflictingCommands = new HashSet<>();
        Set<Subsystem> requirements = command.getRequirements();
        for (Subsystem requirement : activeSubsystems.keySet()) {
            if (requirements.contains(requirement)) {
                conflictingCommands.add(activeSubsystems.get(requirement));
            }
        }
        return conflictingCommands;
    }

    private void endCommand(Command command, boolean interrupted) {
        if (!activeCommands.contains(command)) return;

        Set<Subsystem> requirements = command.getRequirements();

        command.stop(interrupted);
        activeCommands.remove(command);
        for (Subsystem requirement : requirements) {
            activeSubsystems.remove(requirement);
        }
    }

    public void endAll() {
        scheduledCommands.clear();
        for (Command command : activeCommands) {
            endCommand(command, true);
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Scheduled commands", getCommandNames(scheduledCommands));
        telemetry.addData("Active commands", getCommandNames(activeCommands));
        telemetry.addData("Active subsystems && commands", getSubsystemCommands(activeSubsystems));
    }

    private String getCommandNames(Set<Command> commands) {
        StringBuilder listOfNames = new StringBuilder();
        for (Command command : commands) {
            listOfNames.append(command.getName());
            listOfNames.append(", ");
        }
        listOfNames.append(";  ");
        return listOfNames.toString();
    }

    private String getSubsystemCommands(LinkedHashMap<Subsystem, Command> subsystems) {
        StringBuilder listOfNames = new StringBuilder();
        for (Map.Entry<Subsystem, Command> subsystemSet : subsystems.entrySet()) {
            listOfNames.append(subsystemSet.getKey().getName());
            listOfNames.append(": ");
            listOfNames.append(subsystemSet.getValue().getName());
            listOfNames.append(", ");
        }
        listOfNames.append("-;- ");
        return listOfNames.toString();
    }

}
