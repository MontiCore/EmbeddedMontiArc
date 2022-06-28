package de.thesis.provider.backend;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.reactive.function.client.WebClient;

@Configuration
public class CompositionConfig {

	@Bean
	public WebClient getWebClient(@Value("${consumer.url}") String consumerUrl) {
		return WebClient.create(consumerUrl);
	}
}
