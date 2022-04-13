package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Dataset;
import de.thesis.consumer.backend.exceptions.DatasetNotFoundException;

import java.util.List;
import java.util.UUID;

public interface DatasetRepository {

	void save(Dataset dataset);

	List<Dataset> findAll();

	Dataset findById(UUID id) throws DatasetNotFoundException;
}
