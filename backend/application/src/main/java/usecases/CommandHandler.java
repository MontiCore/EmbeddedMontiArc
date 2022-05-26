package usecases;

import commands.Command;

public interface CommandHandler<C extends Command, T> {
	T handle(C command);
}
