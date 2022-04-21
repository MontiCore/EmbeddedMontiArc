package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.model.DataRow;
import de.thesis.consumer.backend.domain.repository.DataRowRepository;
import de.thesis.consumer.backend.persistence.entity.DataRowEntity;
import de.thesis.consumer.backend.persistence.entity.DatasetEntity;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

@Repository
@AllArgsConstructor
public class DataRowRepositoryImpl implements DataRowRepository {

	private final SpringDataDataRowCrudRepository dataRowRepo;
	private final SpringDataDatasetCrudRepository datasetRepo;
	private final ObjectMapper mapper;

	@Override
	public void save(DataRow row) {
		DataRowEntity entity = mapToEntity(row);
		dataRowRepo.save(entity);
	}

	@Override
	public List<DataRow> findAll() {
		List<DataRowEntity> entities = dataRowRepo.findAll();
		return entities.stream()
				.map(entity -> mapper.convertValue(entity, DataRow.class))
				.collect(Collectors.toList());
	}

	private DataRowEntity mapToEntity(DataRow row) {
		DataRowEntity entity = mapper.convertValue(row, DataRowEntity.class);

		if (row.getDataset() != null) {
			Optional<DatasetEntity> dataset = datasetRepo.findById(row.getDataset().getId());
			dataset.ifPresent(entity::setDataset);
		}


		return entity;
	}
}
