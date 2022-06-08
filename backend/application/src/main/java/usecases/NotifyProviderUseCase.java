package usecases;

import commands.NotifyProviderCommand;
import lombok.AllArgsConstructor;
import ports.ProviderCommunicationPort;

@AllArgsConstructor
public class NotifyProviderUseCase implements CommandHandler<NotifyProviderCommand> {

	private final ProviderCommunicationPort providerCommunicationPort;

	@Override
	public void handle(NotifyProviderCommand command) {
		providerCommunicationPort.notifyProvider(command.getUrl(), command.getDatasetId());
	}
}
