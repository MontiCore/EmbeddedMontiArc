package persistence.mappers;

import com.fasterxml.jackson.databind.ObjectMapper;
import entity.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;
import persistence.entity.DatasetEntity;

@Component
@AllArgsConstructor
public class JacksonDatasetMapper implements Mapper<Dataset, DatasetEntity> {

	private final ObjectMapper mapper;

	@Override
	public DatasetEntity mapTo(Dataset dataset) {
		return mapper.convertValue(dataset, DatasetEntity.class);
	}

	@Override
	public Dataset mapFrom(DatasetEntity datasetEntity) {
		return mapper.convertValue(datasetEntity, Dataset.class);
	}
}
