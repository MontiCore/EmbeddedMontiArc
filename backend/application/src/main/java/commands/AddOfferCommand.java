package commands;

import entity.DataRow;
import entity.Metadata;
import entity.Policy;
import lombok.Value;

import java.time.LocalDate;
import java.util.List;

@Value
public class AddOfferCommand implements Command {
	Metadata metadata;
	List<DataRow> data;
}
