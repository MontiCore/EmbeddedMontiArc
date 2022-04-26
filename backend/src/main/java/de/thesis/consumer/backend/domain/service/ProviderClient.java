package de.thesis.consumer.backend.domain.service;

import de.thesis.consumer.backend.domain.model.Dataset;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class ProviderClient {

	private final IHttpClient httpClient;

	public void notifyDataOwner(Dataset dataset) {
		// TODO fix path and response
		httpClient.post(dataset.getProvider(), dataset.getId(), Object.class, null);
	}
}
