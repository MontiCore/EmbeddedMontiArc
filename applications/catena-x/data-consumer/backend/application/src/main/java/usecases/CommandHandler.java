package usecases;

import commands.Command;

public interface CommandHandler<C extends Command> {
	void handle(C command);
}
