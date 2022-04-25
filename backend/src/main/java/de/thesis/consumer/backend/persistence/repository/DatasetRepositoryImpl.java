package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.exception.DatasetNotFoundException;
import de.thesis.consumer.backend.domain.exception.PolicyNotFoundException;
import de.thesis.consumer.backend.domain.model.DataRow;
import de.thesis.consumer.backend.domain.model.Dataset;
import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.persistence.entity.DataRowEntity;
import de.thesis.consumer.backend.persistence.entity.DatasetEntity;
import de.thesis.consumer.backend.persistence.entity.PolicyEntity;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.UUID;
import java.util.stream.Collectors;

@Service
@AllArgsConstructor
public class DatasetRepositoryImpl implements DatasetRepository {

	private final SpringDataDatasetCrudRepository datasetRepository;
	private final SpringDataDataRowCrudRepository dataRowRepository;
	private final SpringDataPolicyCrudRepository policyRepository;
	private final ObjectMapper mapper;

	@Override
	public void save(Dataset dataset) throws PolicyNotFoundException {
		PolicyEntity policyEntity = policyRepository.findById(dataset.getPolicy().getId()).orElseThrow(PolicyNotFoundException::new);
		List<DataRowEntity> rows = dataRowRepository.findAllByOfferId(dataset.getId());
		DatasetEntity entity = mapper.convertValue(dataset, DatasetEntity.class);
		for(DataRowEntity dataRow : rows) {
			dataRow.setDataset(entity);
		}
		entity.setPolicy(policyEntity);
		entity.setData(rows);
		datasetRepository.save(entity);
	}

	@Override
	public List<Dataset> findAll() {
		List<DatasetEntity> entities = datasetRepository.findAll();
		return entities.stream()
				.map(entity -> {
					Dataset dataset = mapper.convertValue(entity, Dataset.class);
					dataset.setPolicy(mapper.convertValue(entity.getPolicy(), Policy.class));
					return dataset;
				})
				.collect(Collectors.toList());
	}

	@Override
	public Dataset findById(UUID id) throws DatasetNotFoundException {
		DatasetEntity entity = datasetRepository.findById(id).orElseThrow(() -> new DatasetNotFoundException("Dataset not found"));
		Dataset dataset = mapper.convertValue(entity, Dataset.class);
		dataset.setData(entity.getData().stream().map(dataRowEntity -> mapper.convertValue(dataRowEntity, DataRow.class)).collect(Collectors.toList()));

		return dataset;
	}

	@Override
	public void deleteById(UUID id) {
		datasetRepository.deleteById(id);
	}
}
