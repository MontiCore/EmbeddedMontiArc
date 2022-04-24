package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.exception.PolicyNotFoundException;
import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.persistence.entity.OfferEntity;
import de.thesis.consumer.backend.persistence.entity.PolicyEntity;
import de.thesis.consumer.backend.persistence.mapper.OfferMapper;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;
import java.util.UUID;
import java.util.stream.Collectors;

@Repository
@AllArgsConstructor
public class OfferRepositoryImpl implements OfferRepository {

	private SpringDataOfferCrudRepository offerRepository;
	private SpringDataPolicyCrudRepository policyRepository;
	private final OfferMapper mapper;
	private final ObjectMapper objectMapper;

	@Override
	public void save(Offer offer) throws PolicyNotFoundException {
		PolicyEntity policyEntity = policyRepository.findById(offer.getPolicy().getId())
				.orElse(new PolicyEntity(offer.getPolicy().getId(), offer.getPolicy().getRawValue()));
		policyRepository.save(policyEntity);
		OfferEntity entity = mapper.mapTo(offer);
		entity.setPolicy(policyEntity);

		offerRepository.save(entity);
	}

	@Override
	public List<Offer> findAll() {
		List<OfferEntity> entities = offerRepository.findAll();
		return entities.stream()
				.map(entity -> {
					Offer offer = mapper.mapFrom(entity);
					offer.setPolicy(objectMapper.convertValue(offer.getPolicy(), Policy.class));
					return offer;
				})
				.collect(Collectors.toList());
	}

	@Override
	public Offer findBy(UUID offerId) {
		OfferEntity entity = offerRepository.findById(offerId).orElse(null);
		Offer offer = mapper.mapFrom(entity);
		offer.setPolicy(objectMapper.convertValue(entity.getPolicy(), Policy.class));

		return offer;
	}
}
