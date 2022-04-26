package de.thesis.consumer.backend.domain.service;

import org.springframework.util.MultiValueMap;

public interface IHttpClient {

	<T> T post(String path,
			   Object requestBody,
			   Class<T> responseType,
			   MultiValueMap<String, String> headers);
}
