package de.thesis.provider.backend;


import de.thesis.provider.backend.policy.Policy;
import lombok.Getter;
import lombok.Setter;

import java.util.UUID;

@Getter
@Setter
public class DatasetCreateCommand {
	private UUID id;
	private Policy policy;
	private MetaData metaData;
	private String file;
	private String rows;
}
