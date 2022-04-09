package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Offer;
import lombok.AllArgsConstructor;

import java.util.List;

@AllArgsConstructor
public class OfferService {

	private OfferRepository offerRepository;
	private PolicyService policyService;

	public void offerDataset(Offer offer) throws InvalidPolicyException {
		if (!policyService.isValid(offer.getPolicy())) {
			throw new InvalidPolicyException("Policy invalid");
		}

		offerRepository.save(offer);
	}

	public List<Offer> getAllOffers() {
		return offerRepository.findAll();
	}
}
