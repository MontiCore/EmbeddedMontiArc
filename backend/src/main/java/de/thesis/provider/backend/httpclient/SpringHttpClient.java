package de.thesis.provider.backend.httpclient;

import de.thesis.provider.backend.httpclient.HttpClient;
import lombok.AllArgsConstructor;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Component;
import org.springframework.util.MultiValueMap;
import org.springframework.web.reactive.function.client.WebClient;

import java.util.Map;

@Component
@AllArgsConstructor
public class SpringHttpClient implements HttpClient {

	private final WebClient webClient;

	@Override
	public <S, T> Map<S, T> get(String path, MultiValueMap<String, String> queryParams, MultiValueMap<String, String> headers) {

		return webClient.get()
				.uri(builder -> builder.path(path).queryParams(queryParams).build())
				.headers(httpHeaders -> {
					if (headers != null) {
						httpHeaders.addAll(headers);
					}
				})
				.retrieve()
				.bodyToMono(Map.class)
				.block();
	}

	@Override
	public <T> T post(String path, Object requestBody, Class<T> responseType, MultiValueMap<String, String> headers) {
		return webClient.post()
				.uri(path)
				.headers(httpHeaders -> {
					if (headers != null) {
						httpHeaders.addAll(headers);
					}
				})
				.contentType(MediaType.APPLICATION_JSON)
				.bodyValue(requestBody)
				.retrieve()
				.bodyToMono(responseType)
				.block();
	}

	@Override
	public <T> T put(String path, Object requestBody, Class<T> responseType, MultiValueMap<String, String> headers) {
		return webClient.put()
				.uri(path)
				.headers(httpHeaders -> {
					if (headers != null) {
						httpHeaders.addAll(headers);
					}
				})
				.contentType(MediaType.APPLICATION_JSON)
				.bodyValue(requestBody)
				.retrieve()
				.bodyToMono(responseType)
				.block();
	}

	@Override
	public <T> T delete(String path, Class<T> responseType, MultiValueMap<String, String> headers) {
		return webClient.delete()
				.uri(path)
				.headers(httpHeaders -> {
					if (headers != null) {
						httpHeaders.addAll(headers);
					}
				})
				.retrieve()
				.bodyToMono(responseType)
				.block();
	}
}
