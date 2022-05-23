package usecase;

import entity.Dataset;
import lombok.AllArgsConstructor;
import port.ProviderCommunicationPort;

@AllArgsConstructor
public class NotifyProviderUseCase {

	private final ProviderCommunicationPort providerCommunicationPort;

	public void notifyDataOwner(Dataset dataset) {
		providerCommunicationPort.notifyProvider(dataset);
	}
}
