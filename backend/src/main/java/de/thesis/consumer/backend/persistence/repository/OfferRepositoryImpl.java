package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.persistence.entity.OfferEntity;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.UUID;
import java.util.stream.Collectors;

@Repository
@AllArgsConstructor
public class OfferRepositoryImpl implements OfferRepository {

	private SpringDataOfferCrudRepository repository;
	private final ObjectMapper mapper;

	@Override
	public void save(Offer offer) {
		OfferEntity entity = mapper.convertValue(offer, OfferEntity.class);
		repository.save(entity);
	}

	@Override
	public List<Offer> findAll() {
		List<OfferEntity> entities = repository.findAll();
		return entities.stream()
				.map(entity -> mapper.convertValue(entity, Offer.class))
				.collect(Collectors.toList());
	}

	@Override
	public Offer findBy(UUID offerId) {
		OfferEntity entity = repository.findById(offerId).orElse(null);
		return mapper.convertValue(entity, Offer.class);
	}
}
