package datasovereignty;

import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;
import ports.HttpClientPort;
import ports.ProviderCommunicationPort;

@Component
@AllArgsConstructor
public class ProviderCommunicationPortAdapter implements ProviderCommunicationPort {

	private final HttpClientPort httpClientPort;

	@Override
	public void notifyProvider(String url, String datasetId) {
		httpClientPort.post(url + "/" + datasetId, datasetId, Object.class);
	}
}
