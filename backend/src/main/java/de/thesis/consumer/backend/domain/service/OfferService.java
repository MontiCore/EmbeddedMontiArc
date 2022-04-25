package de.thesis.consumer.backend.domain.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.fraunhofer.iese.mydata.exception.ConflictingResourceException;
import de.fraunhofer.iese.mydata.exception.InvalidEntityException;
import de.fraunhofer.iese.mydata.exception.NoSuchEntityException;
import de.fraunhofer.iese.mydata.exception.ResourceUpdateException;
import de.thesis.consumer.backend.domain.IPolicyManagementPoint;
import de.thesis.consumer.backend.domain.PolicyInstantiationException;
import de.thesis.consumer.backend.domain.exception.InvalidPolicyException;
import de.thesis.consumer.backend.domain.exception.PolicyNotFoundException;
import de.thesis.consumer.backend.domain.model.DataRow;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.repository.DataRowRepository;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.domain.repository.PolicyRepository;
import lombok.AllArgsConstructor;

import java.io.IOException;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@AllArgsConstructor
public class OfferService {

	private OfferRepository offerRepository;
	private PolicyRepository policyRepository;
	private DatasetRepository datasetRepository;
	private DataRowRepository dataRowRepository;
	private IPolicyService IPolicyService;
	private IPolicyManagementPoint pmp;
	private ObjectMapper mapper;

	public void offer(Offer offer) throws InvalidPolicyException, PolicyNotFoundException {
		if (!IPolicyService.isValid(offer.getPolicy())) {
			throw new InvalidPolicyException("Policy invalid");
		}

		policyRepository.save(offer.getPolicy());
		offerRepository.save(offer);
	}

	public List<Offer> getAllOffers() {
		return offerRepository.findAll();
	}

	public void buyOffer(UUID offerId) throws PolicyInstantiationException, ConflictingResourceException, IOException, NoSuchEntityException, InvalidEntityException, ResourceUpdateException, PolicyNotFoundException {
		Offer offer = offerRepository.findBy(offerId);
		pmp.instantiatePolicy(offer.getPolicy());
		Dataset dataset = mapper.convertValue(offer, Dataset.class);
		dataset.setBoughtAt(LocalDateTime.now());
		List<DataRow> rows = dataRowRepository.findAll();
		datasetRepository.save(dataset);
		for (DataRow row : rows) {
			row.setDataset(dataset);
			dataRowRepository.save(row);
		}
	}
}
