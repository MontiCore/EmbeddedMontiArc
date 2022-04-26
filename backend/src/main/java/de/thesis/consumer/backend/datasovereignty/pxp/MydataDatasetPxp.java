package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import lombok.AllArgsConstructor;

@PxpService(componentName = "dataset-pxp")
@AllArgsConstructor
public class MydataDatasetPxp {

	// private final DatasetRepository repository;

	@ActionDescription(methodName = "delete-dataset")
	public boolean deleteDataset(@ActionParameterDescription(name = "datasetId", mandatory = true) final String datasetId) {
		System.err.println("Executing pxp with id: " + datasetId);
		// repository.deleteById(UUID.fromString(id));

		return true;
	}
}
