package datasovereignty;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetDeletionExecutionPoint;
import ports.DatasetPersistencePort;

import java.util.UUID;

@PxpService(componentName = "dataset-deletion-pxp")
@AllArgsConstructor
@Slf4j
public class DatasetDeletionExecutionPointAdapter implements DatasetDeletionExecutionPoint {

	private final DatasetPersistencePort datasetPersistencePort;

	@Override
	@ActionDescription(methodName = "delete-dataset")
	public boolean deleteDataset(@ActionParameterDescription(name = "id", mandatory = true) final String datasetId) {
		datasetPersistencePort.deleteById(UUID.fromString(datasetId));
		log.info("Successfully deleted dataset {}", datasetId);

		return true;
	}


	@Override
	public String getName() {
		return "dataset-deletion-pxp";
	}
}
