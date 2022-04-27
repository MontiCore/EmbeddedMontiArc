package de.thesis.consumer.backend.domain.service;

import lombok.AllArgsConstructor;

import java.util.UUID;

@AllArgsConstructor
public class ProviderClient {

	private final IHttpClient httpClient;

	public void notifyDataOwner(String url, UUID datasetId) {
		httpClient.post(url + "/" + datasetId, datasetId, Object.class, null);
	}
}
