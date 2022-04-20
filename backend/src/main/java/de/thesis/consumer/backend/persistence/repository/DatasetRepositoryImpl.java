package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.exception.DatasetNotFoundException;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.persistence.entity.DatasetEntity;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.UUID;
import java.util.stream.Collectors;

@Service
@AllArgsConstructor
public class DatasetRepositoryImpl implements DatasetRepository {

	private final SpringDataDatasetCrudRepository repository;
	private final ObjectMapper mapper;

	@Override
	public void save(Dataset dataset) {
		DatasetEntity entity = mapper.convertValue(dataset, DatasetEntity.class);
		repository.save(entity);
	}

	@Override
	public List<Dataset> findAll() {
		List<DatasetEntity> entities = repository.findAll();
		return entities.stream()
				.map(entity -> mapper.convertValue(entity, Dataset.class))
				.collect(Collectors.toList());
	}

	@Override
	public Dataset findById(UUID id) throws DatasetNotFoundException {
		DatasetEntity entity = repository.findById(id).orElseThrow(() -> new DatasetNotFoundException("Dataset not found"));
		return mapper.convertValue(entity, Dataset.class);
	}
}
