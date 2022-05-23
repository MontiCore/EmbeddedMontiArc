package usecase;

import entity.Dataset;
import lombok.AllArgsConstructor;
import port.DatasetPersistencePort;
import port.PolicyEnforcementPort;

@AllArgsConstructor
public class GetDatasetMetadataUseCase {


	private final DatasetPersistencePort datasetPersistencePort;
	private final PolicyEnforcementPort<Dataset> enforcementPort;

	public Iterable<Dataset> getAllDatasets() {
		return datasetPersistencePort.findAll();
	}
}
