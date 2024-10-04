package com.team254.frc2024.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

/**
 * A command that runs another command repeatedly as long as a condition is met.
 */
public class WhileCommand extends Command {
    private final Command m_command;
    private final BooleanSupplier m_supplier;
    private boolean m_ended;
    private boolean m_while_ended;

    /**
     * Creates a new while. Will run another command similar to a while loop
     *
     * @param command the command to run repeatedly
     */
    @SuppressWarnings("this-escape")
    public WhileCommand(Command command, BooleanSupplier supplier) {
        m_command = requireNonNullParam(command, "command", "WhileCommand");
        CommandScheduler.getInstance().registerComposedCommands(command);
        m_requirements.addAll(command.getRequirements());
        m_supplier = supplier;
        setName("While(" + command.getName() + ")");
    }

    @Override
    public void initialize() {
        m_ended = true;
        m_while_ended = false;
    }

    @Override
    public void execute() {
        if (m_while_ended)
            return;
        if (m_ended) {
            // Check condition
            if (!m_supplier.getAsBoolean()) {
                m_while_ended = true;
                return;
            }
            m_ended = false;
            m_command.initialize();
        }
        m_command.execute();
        if (m_command.isFinished()) {
            // restart command
            m_command.end(false);
            m_ended = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_while_ended;
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure we didn't already call end() (which would happen if the command
        // finished in the
        // last call to our execute())
        if (!m_ended) {
            m_command.end(interrupted);
            m_ended = true;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_command.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return m_command.getInterruptionBehavior();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("command", m_command::getName, null);
    }

}
