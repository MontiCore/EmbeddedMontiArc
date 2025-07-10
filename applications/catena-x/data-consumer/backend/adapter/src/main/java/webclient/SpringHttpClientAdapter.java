package webclient;

import lombok.AllArgsConstructor;
import org.springframework.http.MediaType;
import org.springframework.stereotype.Component;
import org.springframework.web.reactive.function.client.WebClient;
import ports.HttpClientPort;


@Component
@AllArgsConstructor
public class SpringHttpClientAdapter implements HttpClientPort {

	private final WebClient webClient;

	@Override
	public <T> T post(String path, Object requestBody, Class<T> responseType) {
		return webClient.post()
				.uri(path)
				.contentType(MediaType.APPLICATION_JSON)
				.bodyValue(requestBody)
				.retrieve()
				.bodyToMono(responseType)
				.block();
	}
}

