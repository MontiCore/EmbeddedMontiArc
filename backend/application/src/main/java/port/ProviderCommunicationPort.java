package port;

import entity.Dataset;

public interface ProviderCommunicationPort {
	void notifyProvider(String url, String datasetId);
}
