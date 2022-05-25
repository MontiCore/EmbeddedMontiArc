package de.thesis.consumer;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;

@SpringBootApplication
@ComponentScan(basePackages = {"presentation.controllers", "persistence", "de.thesis.consumer.config"})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan(basePackages = "persistence.entity")
public class ConsumerSpringBootApplication {

	public static void main(String[] args) {
		SpringApplication.run(ConsumerSpringBootApplication.class, args);
	}

}
