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
import persistence.entity.DatasetEntity;
import persistence.mappers.JacksonDatasetMapper;
import persistence.mappers.JacksonOfferMapper;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;

@DataJpaTest
@ContextConfiguration(classes = {SpringDatasetRepositoryPortAdapter.class, SpringOfferPersistencePortAdapter.class, JacksonDatasetMapper.class, JacksonOfferMapper.class, SpringDatasetRepository.class, SpringOfferRepository.class, SpringOfferRepository.class, ObjectMapper.class})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan("persistence.entity")
class SpringDatasetRepositoryPortAdapterTest {

	@Autowired
	private SpringDatasetRepositoryPortAdapter underTest;

	@Autowired
	private SpringOfferPersistencePortAdapter offerRepository;

	@Autowired
	private SpringDatasetRepository springDatasetRepository;

	@AfterEach
	private void cleanUpEach() {
		springDatasetRepository.deleteAll();
	}

	@Test
	void shouldSaveDataset() {
		// given
		Policy policy = new Policy();
		policy.setMaxUsages(3);

		Metadata metadata = new Metadata();
		metadata.setTitle("Aachen Dataset");
		metadata.setProvider("Carrier GmbH");
		metadata.setDescription("A simple test data set...");
		metadata.setPrice(10);
		metadata.setLoggingUrl("/logging");
		metadata.setPolicy(policy);

		DataRow datarow = new DataRow();
		datarow.setDayID("1234");
		datarow.setLongitude(51);
		datarow.setLatitude(10);
		datarow.setGpsTime(null);
		datarow.setHeading(42);
		datarow.setSpeed(50);
		datarow.setOdometer(22000);
		datarow.setTotalFuelUsed(50);
		datarow.setTimestamp(null);

		UUID offerId = UUID.randomUUID();
		Offer offer = new Offer(offerId, metadata, List.of(datarow));
		offerRepository.save(offer);
		Offer savedOffer = offerRepository.findBy(offerId);

		// when
		UUID datasetId = UUID.randomUUID();
		Dataset dataset = new Dataset();
		dataset.setId(datasetId);
		dataset.setOffer(savedOffer);
		dataset.setMetadata(savedOffer.getMetadata());
		dataset.setData(savedOffer.getData());
		dataset.setBoughtAt(LocalDateTime.now());

		underTest.save(dataset);

		// then
		Optional<DatasetEntity> datasetEntity = springDatasetRepository.findById(datasetId);
		assertThat(datasetEntity.isPresent()).isTrue();
	}
}
