package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Dataset;

public interface DatasetRepository {

	void save(Dataset dataset);
}
