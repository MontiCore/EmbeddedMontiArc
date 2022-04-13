package de.thesis.consumer.backend.domain;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.entities.Dataset;
import de.thesis.consumer.backend.entities.Offer;
import de.thesis.consumer.backend.entities.TruckData;
import lombok.AllArgsConstructor;

import java.util.List;
import java.util.UUID;

@AllArgsConstructor
public class OfferService {

	private OfferRepository offerRepository;
	private DatasetRepository datasetRepository;
	private TruckDataRepository truckDataRepository;
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
		List<TruckData> truckData = truckDataRepository.findAll();
		dataset.setTruckData(truckData);
		datasetRepository.save(dataset);
		for(TruckData td : truckData) {
			td.setDataset(dataset);
			truckDataRepository.save(td);
		}
	}
}
