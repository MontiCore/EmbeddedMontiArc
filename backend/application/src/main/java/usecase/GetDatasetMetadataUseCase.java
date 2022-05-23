package usecase;

import entity.Dataset;
import lombok.AllArgsConstructor;
import port.DatasetPersistencePort;

@AllArgsConstructor
public class GetDatasetMetadataUseCase {

	private final DatasetPersistencePort datasetPersistencePort;

	public Iterable<Dataset> getDatasetsMetadata() {
		return datasetPersistencePort.findAll();
	}
}
