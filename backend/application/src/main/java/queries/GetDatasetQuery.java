package queries;

import lombok.Value;

import java.util.UUID;

@Value
public class GetDatasetQuery implements Query {
	UUID datasetId;
}
