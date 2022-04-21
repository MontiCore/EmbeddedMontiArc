package de.thesis.consumer.backend.domain.service;

import de.thesis.consumer.backend.domain.InhibitionException;
import de.thesis.consumer.backend.domain.PolicyEnforcementPoint;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.exception.DatasetNotFoundException;
import lombok.AllArgsConstructor;

import java.util.List;
import java.util.UUID;

@AllArgsConstructor
public class DatasetService {

	private final DatasetRepository repository;
	private final PolicyEnforcementPoint<Dataset> pep;

	public List<Dataset> getAllDatasets() {
		return repository.findAll();
	}

	public Dataset getDataset(UUID id) throws DatasetNotFoundException, InhibitionException {
		Dataset dataset = repository.findById(id);

		return pep.enforce(dataset);
	}
}
