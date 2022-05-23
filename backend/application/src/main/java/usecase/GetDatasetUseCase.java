package usecase;

import entity.Dataset;
import lombok.AllArgsConstructor;
import port.DatasetPersistencePort;
import port.PolicyEnforcementPort;

import java.util.UUID;

@AllArgsConstructor
public class GetDatasetUseCase {

	private final DatasetPersistencePort datasetPersistencePort;
	private final PolicyEnforcementPort<Dataset> enforcementPort;

	public Dataset getDataset(UUID id) {
		Dataset dataset = datasetPersistencePort.findById(id);
		return enforcementPort.enforce(dataset);
	}
}
