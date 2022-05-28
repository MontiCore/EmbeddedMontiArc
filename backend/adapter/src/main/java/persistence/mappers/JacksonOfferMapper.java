package persistence.mappers;

import com.fasterxml.jackson.databind.ObjectMapper;
import entity.Offer;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;
import persistence.entity.OfferEntity;

@Component
@AllArgsConstructor
public class JacksonOfferMapper implements OfferMapper<OfferEntity> {

	private final ObjectMapper mapper;

	@Override
	public OfferEntity mapToPersistenceEntity(Offer offer) {
		return mapper.convertValue(offer, OfferEntity.class);
	}

	@Override
	public Offer mapToDomainEntity(OfferEntity offerEntity) {
		return mapper.convertValue(offerEntity, Offer.class);
	}
}
