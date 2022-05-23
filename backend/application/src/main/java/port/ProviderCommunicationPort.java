package port;

import entity.Dataset;

public interface ProviderCommunicationPort {
	void notifyProvider(Dataset dataset);
}
