/*******************************************************************************
 *    Copyright 2015, 2016 Taylor G Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *******************************************************************************/
package de.monticore.lang.monticar.generator.middleware.helpers.AffinityPropagationHelper.clust4j.algo.preprocess;

import de.monticore.lang.monticar.generator.middleware.helpers.AffinityPropagationHelper.clust4j.Clust4j;
import de.monticore.lang.monticar.generator.middleware.helpers.AffinityPropagationHelper.clust4j.utils.DeepCloneable;
import de.monticore.lang.monticar.generator.middleware.helpers.AffinityPropagationHelper.clust4j.utils.SynchronicityLock;
import de.monticore.lang.monticar.generator.middleware.helpers.AffinityPropagationHelper.clust4j.utils.TableFormatter;
import org.apache.commons.math3.linear.RealMatrix;

import de.monticore.lang.monticar.generator.middleware.helpers.AffinityPropagationHelper.clust4j.algo.BaseModel;

public abstract class PreProcessor extends Clust4j implements DeepCloneable {
	private static final long serialVersionUID = -312158525538380532L;
	final public static TableFormatter formatter = BaseModel.formatter;
	
	/** The lock to synchronize on for fits */
	protected final Object fitLock = new SynchronicityLock();
	
	@Override public abstract PreProcessor copy();
	public abstract PreProcessor fit(RealMatrix X);
	public abstract RealMatrix transform(RealMatrix data);
	public abstract double[][] transform(double[][] data);
}
