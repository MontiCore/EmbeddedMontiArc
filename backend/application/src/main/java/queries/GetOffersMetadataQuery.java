package queries;

import lombok.Value;

@Value
public class GetOffersMetadataQuery implements Query {
	boolean onlyBought;
}

