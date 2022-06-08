package ports;

import java.util.UUID;

public interface ProviderCommunicationPort {
	void notifyProvider(String url, UUID datasetId);
}
