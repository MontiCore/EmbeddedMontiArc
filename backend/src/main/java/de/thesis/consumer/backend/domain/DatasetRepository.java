package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Dataset;

import java.util.List;

public interface DatasetRepository {

	void save(Dataset dataset);

	List<Dataset> findAll();
}
