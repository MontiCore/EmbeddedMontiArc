package integration;

import datasovereignty.ProviderCommunicationPortAdapter;
import org.junit.jupiter.api.Test;
import org.springframework.web.reactive.function.client.WebClient;
import webclient.SpringHttpClientAdapter;

import java.util.UUID;

public class NotifyProviderTest {

	@Test
	public void doTest() {
		WebClient webClient = WebClient.create("http://localhost:8082");
		ProviderCommunicationPortAdapter providerCommunicationPortAdapter = new ProviderCommunicationPortAdapter(new SpringHttpClientAdapter(webClient));
		providerCommunicationPortAdapter.notifyProvider("logging", UUID.fromString("f10d7f03-ae67-4c51-ba92-ab62f7962973"));
	}
}
