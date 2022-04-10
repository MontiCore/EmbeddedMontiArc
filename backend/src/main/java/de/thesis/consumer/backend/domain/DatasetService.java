package de.thesis.consumer.backend.domain;

import de.thesis.consumer.backend.entities.Dataset;
import lombok.AllArgsConstructor;

import java.util.List;

@AllArgsConstructor
public class DatasetService {

	private final DatasetRepository repository;

	public List<Dataset> getAllDatasets() {
		return repository.findAll();
	}
}
