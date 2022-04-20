package de.thesis.consumer.backend.persistence.repository;

import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.persistence.entity.Dataset;
import de.thesis.consumer.backend.domain.exception.DatasetNotFoundException;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.UUID;

@Service
@AllArgsConstructor
public class DatasetRepositoryImpl implements DatasetRepository {

	private final SpringDataDatasetCrudRepository repository;

	@Override
	public void save(Dataset dataset) {
		repository.save(dataset);
	}

	@Override
	public List<Dataset> findAll() {
		return repository.findAll();
	}

	@Override
	public Dataset findById(UUID id) throws DatasetNotFoundException{
		return repository.findById(id).orElseThrow(() -> new DatasetNotFoundException("Dataset not found"));
	}
}
