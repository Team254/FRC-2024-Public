package com.team254.frc2024.commands;

import java.util.Map;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

/**
 * Based on WPILib's SelectCommand class
 */
public class DynamicSelectCommand<K> extends Command {
  private final Map<K, Command> m_commands;
  private final Supplier<? extends K> m_selector;
  private Command m_selectedCommand;
  private boolean m_runsWhenDisabled = true;
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  private final Command m_defaultCommand = new PrintCommand(
      "DynamicSelectCommand selector value does not correspond to any command!");

  /**
   * Creates a new DynamicSelectCommand.
   *
   * @param commands the map of commands to choose from
   * @param selector the selector to determine which command to run
   */
  public DynamicSelectCommand(Map<K, Command> commands, Supplier<? extends K> selector) {
    m_commands = requireNonNullParam(commands, "commands", "SelectCommand");
    m_selector = requireNonNullParam(selector, "selector", "SelectCommand");

    CommandScheduler.getInstance().registerComposedCommands(m_defaultCommand);
    CommandScheduler.getInstance()
        .registerComposedCommands(commands.values().toArray(new Command[] {}));

    for (Command command : m_commands.values()) {
      m_requirements.addAll(command.getRequirements());
      m_runsWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  private Command getNextCommand() {
    return m_commands.getOrDefault(m_selector.get(), m_defaultCommand);
  }

  @Override
  public void initialize() {
    m_selectedCommand = getNextCommand();
    m_selectedCommand.initialize();
  }

  @Override
  public void execute() {
    Command currentCommand = m_selectedCommand;
    Command nextCommand = getNextCommand();
    if (currentCommand.equals(nextCommand)) {
      currentCommand.execute();
    } else {
      currentCommand.cancel();
      nextCommand.initialize();
      nextCommand.execute();
    }
    m_selectedCommand = nextCommand;
  }

  @Override
  public void end(boolean interrupted) {
    m_selectedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_selectedCommand.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runsWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty(
        "selected", () -> m_selectedCommand == null ? "null" : m_selectedCommand.getName(), null);
  }
}
