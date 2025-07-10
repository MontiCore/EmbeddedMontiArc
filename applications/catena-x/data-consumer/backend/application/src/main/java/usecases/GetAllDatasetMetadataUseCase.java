package usecases;

import entity.Dataset;
import entity.Metadata;
import lombok.AllArgsConstructor;
import ports.DatasetPersistencePort;
import queries.GetAllDatasetMedataQuery;

import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

@AllArgsConstructor
public class GetAllDatasetMetadataUseCase implements QueryHandler<GetAllDatasetMedataQuery, Map<UUID, Metadata>> {

	private final DatasetPersistencePort datasetPersistencePort;

	@Override
	public Map<UUID, Metadata> handle(GetAllDatasetMedataQuery query) {
		Map<UUID, Metadata> map = new HashMap<>();
		Iterable<Dataset> datasets = datasetPersistencePort.findAll();
		datasets.forEach(dataset -> map.put(dataset.getId(), dataset.getMetadata()));

		return map;
	}
}
