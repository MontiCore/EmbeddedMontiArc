package de.thesis.consumer.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import entity.Policy;
import entity.Timer;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import persistence.mappers.JacksonOfferMapper;
import persistence.repository.SpringOfferRepository;
import persistence.repository.SpringOfferRepositoryAdapter;
import ports.OfferPersistencePort;
import ports.PolicyManagementPort;
import usecases.AddOfferUseCase;

@Configuration
public class CompositionConfig {

	@Bean
	public AddOfferUseCase addOfferUseCase(OfferPersistencePort offerPersistencePort) {
		return new AddOfferUseCase(offerPersistencePort);
	}

//	@Bean
//	public DataRowPersistencePort dataRowPersistencePort(JacksonDataRowMapper mapper, SpringDataRowRepository repository) {
//		return new SpringDataRowRepositoryAdapter(mapper, repository);
//	}

	@Bean
	public OfferPersistencePort offerPersistencePort(JacksonOfferMapper mapper, SpringOfferRepository repository) {
		return new SpringOfferRepositoryAdapter(mapper, repository);
	}

	@Bean
	public JacksonOfferMapper jacksonOfferMapper(ObjectMapper mapper) {
		return new JacksonOfferMapper(mapper);
	}

	@Bean
	PolicyManagementPort policyManagementPort() {
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
