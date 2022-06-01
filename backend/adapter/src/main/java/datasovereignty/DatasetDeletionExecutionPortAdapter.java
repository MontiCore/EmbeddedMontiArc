package datasovereignty;

import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import ports.DatasetDeletionExecutionPort;
import ports.DatasetPersistencePort;

import java.util.UUID;

@Component
@AllArgsConstructor
@Slf4j
public class DatasetDeletionExecutionPortAdapter implements DatasetDeletionExecutionPort {

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
