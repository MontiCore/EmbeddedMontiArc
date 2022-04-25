package de.thesis.consumer.backend.persistence.repository;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.model.DataRow;
import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.repository.DataRowRepository;
import de.thesis.consumer.backend.persistence.entity.DataRowEntity;
import de.thesis.consumer.backend.persistence.entity.OfferEntity;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.UUID;
import java.util.stream.Collectors;

@Repository
@AllArgsConstructor
public class DataRowRepositoryImpl implements DataRowRepository {

	private final SpringDataDataRowCrudRepository dataRowRepo;
	private final SpringDataOfferCrudRepository offerRepo;
	private final ObjectMapper mapper;

	@Override
	public void saveAll(List<DataRow> data) {
		// OfferEntity offer = offerRepo.findById(data.get(0).getOffer().getId()).orElse(null);
		List<DataRowEntity> entities = data.stream().map(row -> {
			DataRowEntity entity = mapper.convertValue(row, DataRowEntity.class);
			return entity;
		}).collect(Collectors.toList());
		dataRowRepo.saveAll(entities);
	}

	@Override
	public List<DataRow> findAllByOfferId(UUID offerId) {
		List<DataRowEntity> entities = dataRowRepo.findAllByOfferId(offerId);
		return entities.stream()
				.map(entity -> mapper.convertValue(entity, DataRow.class))
				.collect(Collectors.toList());
	}
}
