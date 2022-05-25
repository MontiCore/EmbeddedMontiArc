package usecases;

import commands.NotifyProviderCommand;
import lombok.AllArgsConstructor;
import ports.ProviderCommunicationPort;

@AllArgsConstructor
public class NotifyProviderUseCase {

	private final ProviderCommunicationPort providerCommunicationPort;

	public void notifyProvider(NotifyProviderCommand command) {
		providerCommunicationPort.notifyProvider(command.getUrl(), command.getDatasetId());
	}
}
