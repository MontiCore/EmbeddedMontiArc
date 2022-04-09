package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Dataset;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class DatasetService {

	private DatasetRepository datasetRepository;
	private PolicyService policyService;

	public void offerDataset(Dataset dataset) throws InvalidPolicyException {
		if (!policyService.isValid(dataset.getPolicy())) {
			throw new InvalidPolicyException("Policy invalid");
		}

		datasetRepository.save(dataset);
	}
}
