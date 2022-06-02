package commands;

import entity.DataRow;
import entity.Metadata;
import lombok.Value;

import java.util.List;

@Value
public class AddOfferCommand implements Command {
	Metadata metadata;
	List<DataRow> data;
}
