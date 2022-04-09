package de.thesis.consumer.backend.persistence;

import de.thesis.consumer.backend.domain.DatasetRepository;
import de.thesis.consumer.backend.entities.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

@Repository
@AllArgsConstructor
public class DatasetRepositoryImpl implements DatasetRepository {

	private SpringDataDatasetCrudRepository repository;

	@Override
	public void save(Dataset dataset) {
		this.repository.save(dataset);
	}
}
