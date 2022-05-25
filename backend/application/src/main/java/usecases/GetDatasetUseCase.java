package usecases;

import entity.Dataset;
import lombok.AllArgsConstructor;
import ports.DatasetPersistencePort;
import ports.PolicyEnforcementPort;
import queries.GetDatasetQuery;

@AllArgsConstructor
public class GetDatasetUseCase implements QueryHandler<GetDatasetQuery, Dataset> {

	private final DatasetPersistencePort datasetPersistencePort;
	private final PolicyEnforcementPort<Dataset> enforcementPort;

	public Dataset handle(GetDatasetQuery query) {
		Dataset dataset = datasetPersistencePort.findById(query.getDatasetId());
		return enforcementPort.enforce(dataset);
	}
}
