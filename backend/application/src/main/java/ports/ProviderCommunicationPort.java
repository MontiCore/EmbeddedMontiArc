package ports;

public interface ProviderCommunicationPort {
	void notifyProvider(String url, String datasetId);
}
