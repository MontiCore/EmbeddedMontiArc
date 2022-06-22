package de.thesis.provider.backend.service;

import de.thesis.provider.backend.CreateDatasetCommand;
import de.thesis.provider.backend.Dataset;
import de.thesis.provider.backend.InsuranceClient;
import de.thesis.provider.backend.Metadata;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.io.IOException;
import java.util.Map;
import java.util.UUID;

@Component
@AllArgsConstructor
public class DatasetService {

	private final InsuranceClient client;

	public void offerDataset(CreateDatasetCommand command) {
		Dataset dataset = new Dataset();
		dataset.setId(UUID.randomUUID());
		dataset.setMetadata(command.getMetadata());
		dataset.setData(command.getData());

		client.offerDataset(dataset);
	}

	public Map<UUID, Metadata> getDatasets() {
		return client.getAllOffersMetadata();
	}
}
