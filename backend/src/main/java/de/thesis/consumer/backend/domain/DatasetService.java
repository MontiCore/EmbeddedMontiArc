package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Dataset;
import de.thesis.consumer.backend.exceptions.DatasetNotFoundException;
import lombok.AllArgsConstructor;

import java.util.List;
import java.util.UUID;

@AllArgsConstructor
public class DatasetService {

	private final DatasetRepository repository;

	public List<Dataset> getAllDatasets() {
		return repository.findAll();
	}

	public Dataset getDataset(UUID id) throws DatasetNotFoundException {
		return repository.findById(id);
	}
}
