package de.thesis.provider.backend;


import lombok.Value;

@Value
public class CreateDatasetCommand {
	Metadata metadata;
	String file;
	String rows;
}
