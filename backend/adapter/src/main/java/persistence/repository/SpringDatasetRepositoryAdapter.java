package persistence.repository;

import entity.Dataset;
import lombok.AllArgsConstructor;
import persistence.entity.DatasetEntity;
import persistence.mappers.JacksonDatasetMapper;
import ports.DatasetPersistencePort;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

@AllArgsConstructor
public class SpringDatasetRepositoryAdapter implements DatasetPersistencePort {

	private final JacksonDatasetMapper mapper;
	private final SpringDatasetRepository repository;

	@Override
	public void save(Dataset dataset) {
		repository.save(mapper.mapToPersistenceEntity(dataset));
	}

	@Override
	public Dataset findById(UUID id) {
		Optional<DatasetEntity> optional = repository.findById(id);

		if (optional.isEmpty()) {
			return null;
		}

		return mapper.mapToDomainEntity(optional.get());
	}

	@Override
	public Iterable<Dataset> findAll() {
		List<Dataset> datasets = new ArrayList<>();
		repository.findAll().forEach(datasetEntity ->
				datasets.add(mapper.mapToDomainEntity(datasetEntity))
		);

		return datasets;
	}

	@Override
	public void deleteById(UUID id) {
		repository.deleteById(id);
	}
}
