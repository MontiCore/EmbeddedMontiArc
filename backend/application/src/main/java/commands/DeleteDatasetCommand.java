package commands;

import lombok.Value;

import java.util.UUID;

@Value
public class DeleteDatasetCommand implements Command {
	UUID datasetId;
}
