package de.thesis.consumer.backend.domain.repository;

import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.exception.DatasetNotFoundException;

import java.util.List;
import java.util.UUID;

public interface DatasetRepository {

	void save(Dataset dataset);

	List<Dataset> findAll();

	Dataset findById(UUID id) throws DatasetNotFoundException;
}
