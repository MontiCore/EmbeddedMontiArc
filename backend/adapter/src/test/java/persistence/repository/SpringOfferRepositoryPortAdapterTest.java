package persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import entity.DataRow;
import entity.Metadata;
import entity.Offer;
import entity.Policy;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.boot.test.autoconfigure.orm.jpa.DataJpaTest;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;
import org.springframework.test.context.ContextConfiguration;
import persistence.entity.DataRowEntity;
import persistence.entity.MetadataEntity;
import persistence.entity.OfferEntity;
import persistence.entity.PolicyEntity;
import persistence.mappers.JacksonOfferMapper;

import java.util.List;
import java.util.Optional;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;

@DataJpaTest
@ContextConfiguration(classes = {SpringOfferPersistencePortAdapter.class, JacksonOfferMapper.class, SpringDatasetRepository.class, SpringOfferRepository.class, ObjectMapper.class})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan("persistence.entity")
class SpringOfferRepositoryPortAdapterTest {

	@Autowired
	private SpringOfferPersistencePortAdapter underTest;

	@Autowired
	private SpringOfferRepository springOfferRepository;

	@AfterEach
	private void cleanUpEach() {
		springOfferRepository.deleteAll();
	}

	@Test
	void shouldSaveOffer() {
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

		UUID uuid = UUID.randomUUID();
		Offer offer = new Offer(uuid, metadata, List.of(datarow));

		underTest.save(offer);

		Optional<OfferEntity> offerEntity = springOfferRepository.findById(uuid);
		assertThat(offerEntity.isPresent()).isTrue();
		assertThat(offerEntity.get().getData().size()).isEqualTo(1);
	}

	@Test
	void shouldFindCorrectOfferById() {
		PolicyEntity policy = new PolicyEntity();
		policy.setMaxUsages(1);

		MetadataEntity metadata = new MetadataEntity();
		metadata.setTitle("Aachen Dataset");
		metadata.setProvider("Carrier GmbH");
		metadata.setDescription("A simple test data set...");
		metadata.setPrice(10);
		metadata.setLoggingUrl("/logging");
		metadata.setPolicy(policy);

		DataRowEntity datarow = new DataRowEntity();
		datarow.setDayID("1234");
		datarow.setLongitude(51);
		datarow.setLatitude(10);
		datarow.setGpsTime(null);
		datarow.setHeading(42);
		datarow.setSpeed(50);
		datarow.setOdometer(22000);
		datarow.setTotalFuelUsed(50);
		datarow.setTimestamp(null);

		UUID uuid = UUID.randomUUID();
		OfferEntity offerEntity = new OfferEntity(uuid, metadata, List.of(datarow));

		springOfferRepository.save(offerEntity);

		Offer offer = underTest.findById(uuid);

		assertThat(offer.getData().get(0).getDayID()).isEqualTo("1234");
	}

	@Test
	void shouldFindTwoOffers() {
		PolicyEntity firstPolicy = new PolicyEntity();
		firstPolicy.setMaxUsages(1);

		MetadataEntity firstMetadata = new MetadataEntity();
		firstMetadata.setTitle("Aachen Dataset");
		firstMetadata.setProvider("Carrier GmbH");
		firstMetadata.setDescription("A simple test data set...");
		firstMetadata.setPrice(10);
		firstMetadata.setLoggingUrl("/logging");
		firstMetadata.setPolicy(firstPolicy);

		PolicyEntity secondPolicy = new PolicyEntity();
		secondPolicy.setMaxUsages(1);

		MetadataEntity secondMetadata = new MetadataEntity();
		secondMetadata.setTitle("Aachen Dataset");
		secondMetadata.setProvider("Carrier GmbH");
		secondMetadata.setDescription("A simple test data set...");
		secondMetadata.setPrice(10);
		secondMetadata.setLoggingUrl("/logging");
		secondMetadata.setPolicy(secondPolicy);

		DataRowEntity datarow = new DataRowEntity();
		datarow.setDayID("1234");
		datarow.setLongitude(51);
		datarow.setLatitude(10);
		datarow.setGpsTime(null);
		datarow.setHeading(42);
		datarow.setSpeed(50);
		datarow.setOdometer(22000);
		datarow.setTotalFuelUsed(50);
		datarow.setTimestamp(null);

		OfferEntity firstOfferEntity = new OfferEntity(UUID.randomUUID(), firstMetadata, List.of(datarow));
		OfferEntity secondOfferEntity = new OfferEntity(UUID.randomUUID(), secondMetadata, List.of(datarow));

		springOfferRepository.save(firstOfferEntity);
		springOfferRepository.save(secondOfferEntity);

		Iterable<Offer> offers = underTest.findAll();

		//then
		int count = 0;
		for (Offer ignored : offers) {
			count++;
		}

		assertThat(count).isEqualTo(2);
	}

	@Test
	void shouldDeleteCorrectOffer() {
		PolicyEntity policy = new PolicyEntity();
		policy.setMaxUsages(1);

		MetadataEntity metadata = new MetadataEntity();
		metadata.setTitle("Aachen Dataset");
		metadata.setProvider("Carrier GmbH");
		metadata.setDescription("A simple test data set...");
		metadata.setPrice(10);
		metadata.setLoggingUrl("/logging");
		metadata.setPolicy(policy);

		DataRowEntity datarow = new DataRowEntity();
		datarow.setDayID("1234");
		datarow.setLongitude(51);
		datarow.setLatitude(10);
		datarow.setGpsTime(null);
		datarow.setHeading(42);
		datarow.setSpeed(50);
		datarow.setOdometer(22000);
		datarow.setTotalFuelUsed(50);
		datarow.setTimestamp(null);

		UUID uuid = UUID.fromString("75d5288c-fff5-45db-be0e-b3d8bedf985d");
		OfferEntity offer = new OfferEntity(uuid, metadata, List.of(datarow));

		springOfferRepository.save(offer);

		underTest.deleteById(uuid);

		int count = 0;
		for (Offer ignored : underTest.findAll()) {
			count++;
		}

		assertThat(count).isEqualTo(0);
	}
}
