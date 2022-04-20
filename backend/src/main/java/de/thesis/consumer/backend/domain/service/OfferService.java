package de.thesis.consumer.backend.domain.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.model.DataRow;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.domain.repository.DataRowRepository;
import de.thesis.consumer.backend.domain.exception.InvalidPolicyException;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.model.Offer;
import lombok.AllArgsConstructor;

import java.util.List;
import java.util.UUID;

@AllArgsConstructor
public class OfferService {

	private OfferRepository offerRepository;
	private DatasetRepository datasetRepository;
	private DataRowRepository dataRowRepository;
	private PolicyService policyService;
	private ObjectMapper mapper;

	public void offerDataset(Offer offer) throws InvalidPolicyException {
		if (!policyService.isValid(offer.getPolicy())) {
			throw new InvalidPolicyException("Policy invalid");
		}

		offerRepository.save(offer);
	}

	public List<Offer> getAllOffers() {
		return offerRepository.findAll();
	}

	public void buyOffer(UUID offerId) {
		Offer offer = offerRepository.findBy(offerId);
		Dataset dataset = mapper.convertValue(offer, Dataset.class);
		List<DataRow> rows = dataRowRepository.findAll();
		datasetRepository.save(dataset);
		for(DataRow row : rows) {
			row.setDataset(dataset);
			dataRowRepository.save(row);
		}
	}
}
