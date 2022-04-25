package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.exception.PolicyNotFoundException;
import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.persistence.entity.DataRowEntity;
import de.thesis.consumer.backend.persistence.entity.OfferEntity;
import de.thesis.consumer.backend.persistence.entity.PolicyEntity;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.UUID;
import java.util.stream.Collectors;

@Repository
@AllArgsConstructor
public class OfferRepositoryImpl implements OfferRepository {

	private SpringDataOfferCrudRepository offerRepository;
	private SpringDataPolicyCrudRepository policyRepository;
	private SpringDataDataRowCrudRepository dataRowRepository;
	private final ObjectMapper mapper;

	@Override
	public void save(Offer offer) throws PolicyNotFoundException {
		PolicyEntity policyEntity =  policyRepository.findById(offer.getPolicy().getId()).orElseThrow(PolicyNotFoundException::new);

		List<DataRowEntity> rows = offer.getData().stream().map(row -> mapper.convertValue(row, DataRowEntity.class)).collect(Collectors.toList());
		OfferEntity entity = mapper.convertValue(offer, OfferEntity.class);
		entity.setData(null);
		for(DataRowEntity dataRow : rows) {
			dataRow.setOffer(entity);
		}
		entity.setPolicy(policyEntity);

		offerRepository.save(entity);
		dataRowRepository.saveAll(rows);
	}

	@Override
	public List<Offer> findAll() {
		List<OfferEntity> entities = offerRepository.findAll();
		return entities.stream()
				.map(entity -> {
					Offer offer = mapper.convertValue(entity, Offer.class);
					offer.setPolicy(mapper.convertValue(entity.getPolicy(), Policy.class));
					return offer;
				})
				.collect(Collectors.toList());
	}

	@Override
	public Offer findBy(UUID offerId) {
		OfferEntity entity = offerRepository.findById(offerId).orElse(null);
		return mapper.convertValue(entity, Offer.class);
	}
}
