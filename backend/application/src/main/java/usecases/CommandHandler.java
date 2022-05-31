package usecases;

import commands.Command;

public interface CommandHandler<C extends Command, R> {
	R handle(C command);
}
