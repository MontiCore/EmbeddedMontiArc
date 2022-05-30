package de.thesis.provider.backend;

import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.util.List;
import java.util.Map;
import java.util.UUID;

@Component
@AllArgsConstructor
public class InsuranceClient {

	private final HttpClient client;

	public void offerDataset(Dataset dataset) {
		client.post("/offers", dataset, Object.class, null);
	}

	public Map<UUID, Metadata> getAllOffersMetadata() {
		return client.get("/offers", null, null);
	}
}
