package persistence.mappers;

import com.fasterxml.jackson.databind.ObjectMapper;
import entity.Offer;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;
import persistence.entity.OfferEntity;

@Component
@AllArgsConstructor
public class JacksonOfferMapper implements Mapper<Offer, OfferEntity> {

	private final ObjectMapper mapper;

	@Override
	public OfferEntity mapTo(Offer offer) {
		return mapper.convertValue(offer, OfferEntity.class);
	}

	@Override
	public Offer mapFrom(OfferEntity offerEntity) {
		return mapper.convertValue(offerEntity, Offer.class);
	}
}
