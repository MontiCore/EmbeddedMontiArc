package de.thesis.provider.backend;

import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.util.Map;
import java.util.UUID;

@Component
@AllArgsConstructor
public class InsuranceClient {

	private final HttpClient client;

	public void addOffer(Offer offer) {
		client.post("/offers", offer, Object.class, null);
	}

	public Map<UUID, Metadata> getAllOffersMetadata() {
		return client.get("/offers", null, null);
	}

	public void deleteOffer(UUID offerId) {
		client.delete("/offers/" + offerId, Object.class, null);
	}
}
