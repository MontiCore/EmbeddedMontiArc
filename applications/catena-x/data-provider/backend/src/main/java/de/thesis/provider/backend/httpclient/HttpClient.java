package de.thesis.provider.backend.httpclient;

import org.springframework.util.MultiValueMap;

import java.util.Map;

public interface HttpClient {

	<S, T> Map<S, T> get(String path,
						 MultiValueMap<String, String> queryParams,
						 MultiValueMap<String, String> headers);


	<T> T post(String path,
			   Object requestBody,
			   Class<T> responseType,
			   MultiValueMap<String, String> headers);

	<T> T put(String path,
			  Object requestBody,
			  Class<T> responseType,
			  MultiValueMap<String, String> headers);

	<T> T delete(String path,
				 Class<T> responseType,
				 MultiValueMap<String, String> headers);
}
