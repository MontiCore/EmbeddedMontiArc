package de.thesis.provider.backend;

import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

@Component
@AllArgsConstructor
public class InsuranceClient {

	private final HttpClient client;

	public void offerDataset(Offer offer) {
		client.post("/offers", offer, Object.class, null);
	}
}
