package de.thesis.provider.backend.services;

import de.thesis.provider.backend.dto.CreateOfferCommand;
import de.thesis.provider.backend.httpclient.InsuranceClient;
import de.thesis.provider.backend.dto.Metadata;
import de.thesis.provider.backend.dto.Offer;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.util.Map;
import java.util.UUID;

@Component
@AllArgsConstructor
public class OfferService {

	private final InsuranceClient client;

	public void addOffer(CreateOfferCommand command) {
		Offer offer = new Offer();
		offer.setId(UUID.randomUUID());
		offer.setMetadata(command.getMetadata());
		offer.setData(command.getData());

		client.addOffer(offer);
	}

	public Map<UUID, Metadata> getOffers() {
		return client.getAllOffersMetadata();
	}

	public void deleteOffer(UUID offerId) {
		client.deleteOffer(offerId);
	}
}
