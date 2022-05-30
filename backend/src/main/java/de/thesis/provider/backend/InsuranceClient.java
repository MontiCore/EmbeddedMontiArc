package de.thesis.provider.backend;

import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.util.List;

@Component
@AllArgsConstructor
public class InsuranceClient {

	private final HttpClient client;

	public void offerDataset(Dataset dataset) {
		client.post("/offers", dataset, Object.class, null);
	}

	public List<Metadata> getOffers() {
		return client.get("/offers", null, Metadata.class, null);
	}
}
