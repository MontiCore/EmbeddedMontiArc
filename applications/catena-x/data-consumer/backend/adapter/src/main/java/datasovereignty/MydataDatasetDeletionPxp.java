package datasovereignty;

import commands.DeleteDatasetCommand;
import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import usecases.DeleteDatasetUseCase;

import java.util.UUID;

@PxpService(componentName = "dataset-deletion-pxp")
@AllArgsConstructor
@Slf4j
public class MydataDatasetDeletionPxp {

	private final DeleteDatasetUseCase deleteDatasetUseCase;

	@ActionDescription(methodName = "delete-dataset")
	public boolean deleteDataset(@ActionParameterDescription(name = "id", mandatory = true) final String datasetId) {
		deleteDatasetUseCase.handle(new DeleteDatasetCommand(UUID.fromString(datasetId)));

		return true;
	}
}
