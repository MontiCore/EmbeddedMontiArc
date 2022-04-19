package de.thesis.provider.backend;

import org.springframework.util.MultiValueMap;

import java.util.List;
import java.util.Map;

public interface HttpClient {

	<T> T get(String path,
			  MultiValueMap<String, String> queryParams,
			  Class<T> responseType,
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
