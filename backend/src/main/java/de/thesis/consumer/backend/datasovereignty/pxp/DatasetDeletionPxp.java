package de.thesis.consumer.backend.datasovereignty.pxp;

import de.fraunhofer.iese.mydata.pxp.PxpService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import lombok.AllArgsConstructor;

import java.util.UUID;

@PxpService(componentName = "dataset-deletion-pxp")
@AllArgsConstructor
public class DatasetDeletionPxp {

	private final DatasetRepository repository;

	@ActionDescription(methodName = "delete-dataset")
	public boolean deleteDataset(@ActionParameterDescription(name = "id", mandatory = true) final String datasetId) {
		repository.deleteById(UUID.fromString(datasetId));

		return true;
	}
}
