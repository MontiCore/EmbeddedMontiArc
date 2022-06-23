package persistence.repository;

import entity.Dataset;
import lombok.AllArgsConstructor;
import persistence.entity.DatasetEntity;
import persistence.mappers.Mapper;
import ports.DatasetPersistencePort;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

@AllArgsConstructor
public class SpringDatasetPersistencePortAdapter implements DatasetPersistencePort {

	private final Mapper<Dataset, DatasetEntity> mapper;
	private final SpringDatasetRepository repository;

	@Override
	public void save(Dataset dataset) {
		repository.save(dataset.getId(), dataset.getBoughtAt(), dataset.getMetadata().getId(), dataset.getOfferId());
	}

	@Override
	public Dataset findById(UUID id) {
		Optional<DatasetEntity> optional = repository.findById(id);

		if (optional.isEmpty()) {
			return null;
		}

		return mapper.mapFrom(optional.get());
	}

	@Override
	public Iterable<Dataset> findAll() {
		List<Dataset> datasets = new ArrayList<>();
		repository.findAll().forEach(datasetEntity ->
				datasets.add(mapper.mapFrom(datasetEntity))
		);

		return datasets;
	}

	@Override
	public void deleteById(UUID id) {
		repository.deleteById(id);
	}

	@Override
	public void deleteByOfferId(UUID offerId) {
		repository.deleteByOfferId(offerId);
	}
}
