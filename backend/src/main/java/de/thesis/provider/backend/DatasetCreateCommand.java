package de.thesis.provider.backend;


import de.thesis.provider.backend.dto.CreatePolicyDto;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class DatasetCreateCommand {
	private CreatePolicyDto policy;
	private MetaData metaData;
	private String file;
	private String rows;
}
