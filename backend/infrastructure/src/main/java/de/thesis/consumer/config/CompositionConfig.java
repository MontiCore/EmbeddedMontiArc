package de.thesis.consumer.config;

import de.thesis.consumer.Main;
import entity.Policy;
import entity.Timer;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import persistence.repository.OfferRepository;
import persistence.repository.SpringOfferRepository;
import port.OfferPersistencePort;
import port.PolicyManagementPort;
import usecase.AddOfferUseCase;

@Configuration
public class CompositionConfig {

	@Bean
	public AddOfferUseCase addOfferUseCase(OfferPersistencePort offerPersistencePort, PolicyManagementPort policyManagementPort) {
		return new AddOfferUseCase(offerPersistencePort, policyManagementPort);
	}

	@Bean
	public OfferPersistencePort offerPersistencePort(SpringOfferRepository repository) {
		return new OfferRepository(repository);
	}

	@Bean PolicyManagementPort policyManagementPort() {
		return new PolicyManagementPort() {
			@Override
			public boolean isValid(Policy policy) {
				return true;
			}

			@Override
			public void deployPolicy(Policy policy) {

			}

			@Override
			public void deployTimer(Timer timer) {

			}
		};
	}
}
