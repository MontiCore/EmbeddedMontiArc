package de.thesis.provider.backend;


import de.thesis.provider.backend.csv.DataRow;
import lombok.Value;

import java.util.List;

@Value
public class CreateDatasetCommand {
	Metadata metadata;
	List<DataRow> data;
}
