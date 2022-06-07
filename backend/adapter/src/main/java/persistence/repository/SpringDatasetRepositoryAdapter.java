package persistence.repository;

import entity.DataRow;
import entity.Dataset;
import entity.Metadata;
import entity.Offer;
import lombok.AllArgsConstructor;
import org.springframework.jdbc.core.JdbcTemplate;
import persistence.entity.DatasetEntity;
import persistence.mappers.Mapper;
import ports.DatasetPersistencePort;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

@AllArgsConstructor
public class SpringDatasetRepositoryAdapter implements DatasetPersistencePort {

	private final Mapper<Dataset, DatasetEntity> mapper;
	private final SpringDatasetRepository repository;
	private final JdbcTemplate jdbcTemplate ;

	@Override
	public void save(Dataset dataset) {
		jdbcTemplate.update(
				"INSERT INTO dataset (id, offer_id, metadata_id, bought_at) VALUES (?, ?, ?, ?)",
				dataset.getId(),
				dataset.getOffer().getId(),
				dataset.getMetadata().getId(),
				dataset.getBoughtAt()
		);

		jdbcTemplate.update(
				"UPDATE data_row SET dataset = ? WHERE offer = ?",
				dataset.getId(),
				dataset.getOffer().getId()
		);
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
}
