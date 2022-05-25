package usecases;

import entity.Dataset;
import lombok.AllArgsConstructor;
import ports.DatasetPersistencePort;
import queries.GetAllDatasetMedataQuery;

@AllArgsConstructor
public class GetAllDatasetMetadataUseCase implements QueryHandler<GetAllDatasetMedataQuery, Iterable<Dataset>> {

	private final DatasetPersistencePort datasetPersistencePort;

	@Override
	public Iterable<Dataset> handle(GetAllDatasetMedataQuery query) {
		return datasetPersistencePort.findAll();
	}
}
