package usecase;

import dto.NotifyProviderCommand;
import lombok.AllArgsConstructor;
import port.ProviderCommunicationPort;

@AllArgsConstructor
public class NotifyProviderUseCase {

	private final ProviderCommunicationPort providerCommunicationPort;

	public void notifyProvider(NotifyProviderCommand command) {
		providerCommunicationPort.notifyProvider(command.getUrl(), command.getDatasetId());
	}
}
