package persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import entity.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.boot.test.autoconfigure.orm.jpa.DataJpaTest;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;
import org.springframework.test.context.ContextConfiguration;
import persistence.entity.*;
import persistence.mappers.JacksonDatasetMapper;
import persistence.mappers.JacksonOfferMapper;

import java.util.List;
import java.util.Optional;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;

@DataJpaTest
@ContextConfiguration(classes = {SpringDatasetPersistencePortAdapter.class, SpringOfferPersistencePortAdapter.class, JacksonDatasetMapper.class, JacksonOfferMapper.class, SpringDatasetRepository.class, SpringOfferRepository.class, ObjectMapper.class})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan("persistence.entity")
class SpringDatasetPersistencePortAdapterTest {

	@Autowired
	private SpringDatasetPersistencePortAdapter underTest;

	@Autowired
	private SpringOfferPersistencePortAdapter springOfferPersistencePortAdapter;

	@Autowired
	private SpringOfferRepository springOfferRepository;

	@Autowired
	private SpringDatasetRepository springDatasetRepository;

	@AfterEach
	private void cleanUpEach() {
		springOfferRepository.deleteAll();
	}

	@Test
	void shouldSaveDataset() {
		// given
		Policy policy = new Policy();
		policy.setMaxUsages(3);

		Metadata metadata = new Metadata(
				1,
				"Aachen Dataset",
				"Carrier GmbH",
				"A simple test data set...",
				10,
				"/logging",
				policy);

		DataRow datarow = new DataRow(
				1,
				"213421",
				51,
				10,
				null,
				42,
				50,
				20000,
				50,
				null
		);

		UUID offerId = UUID.randomUUID();
		Offer offer = new Offer(offerId, metadata, List.of(datarow));
		springOfferPersistencePortAdapter.save(offer);
		Offer savedOffer = springOfferPersistencePortAdapter.findBy(offerId);

		// when
		UUID datasetId = UUID.randomUUID();
		Dataset dataset = new Dataset();
		dataset.setId(datasetId);
		dataset.setMetadata(savedOffer.getMetadata());
		dataset.setOffer(savedOffer);
		dataset.setData(savedOffer.getData());

		underTest.save(dataset);

		Optional<DatasetEntity> datasetEntity = springDatasetRepository.findById(datasetId);
		assertThat(datasetEntity.isPresent()).isTrue();
		assertThat(datasetEntity.get().getData().size()).isEqualTo(1);
	}
}
