package usecases;

import commands.RemoteLoggingCommand;
import lombok.AllArgsConstructor;
import ports.ProviderCommunicationPort;

import java.util.UUID;

@AllArgsConstructor
public class RemoteLoggingUseCase implements CommandHandler<RemoteLoggingCommand> {

	private final ProviderCommunicationPort providerCommunicationPort;

	@Override
	public void handle(RemoteLoggingCommand command) {
		providerCommunicationPort.notifyProvider(command.getUrl(), command.getDatasetId());
	}
}
