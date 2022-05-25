/*
package factories;

import entity.DataRow;
import entity.Offer;
import entity.Policy;
import lombok.AllArgsConstructor;
import services.PolicyValidator;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.UUID;

@AllArgsConstructor
public class OfferFactory {

	private PolicyValidator policyValidator;

	public Offer create(
			UUID id,
			String title,
			String provider,
			String description,
			double price,
			Policy policy,
			LocalDate expiresOn,
			String loggingUrl,
			List<DataRow> data) {

		if (!policyValidator.isValid(policy)) {
			throw new PolicyInvalidException();
		}

		return new Offer(
				id,
				title,
				provider,
				description,
				price,
				policy,
				expiresOn,
				data,
				loggingUrl
		);
	}
}
*/
