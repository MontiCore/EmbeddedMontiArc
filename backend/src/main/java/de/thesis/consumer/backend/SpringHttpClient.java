package de.thesis.consumer.backend;

import de.thesis.consumer.backend.domain.service.IHttpClient;
import lombok.AllArgsConstructor;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Component;
import org.springframework.util.MultiValueMap;
import org.springframework.web.reactive.function.client.WebClient;

@Component
@AllArgsConstructor
public class SpringHttpClient implements IHttpClient {

	private final WebClient webClient;

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
}
