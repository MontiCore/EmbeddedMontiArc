/* (c) https://github.com/MontiCore/monticore */
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
package com.clust4j.data;

abstract class IrisLoader {
	final static DataSet load() {
		return new DataSet(
			getData(),
			getTarget(),
			getHeaders()
		);
	}
	
	final static double[][] getData() {
		return new double[][]{
			new double[]{5.1,3.5,1.4,0.2},
			new double[]{4.9,3,1.4,0.2},
			new double[]{4.7,3.2,1.3,0.2},
			new double[]{4.6,3.1,1.5,0.2},
			new double[]{5,3.6,1.4,0.2},
			new double[]{5.4,3.9,1.7,0.4},
			new double[]{4.6,3.4,1.4,0.3},
			new double[]{5,3.4,1.5,0.2},
			new double[]{4.4,2.9,1.4,0.2},
			new double[]{4.9,3.1,1.5,0.1},
			new double[]{5.4,3.7,1.5,0.2},
			new double[]{4.8,3.4,1.6,0.2},
			new double[]{4.8,3,1.4,0.1},
			new double[]{4.3,3,1.1,0.1},
			new double[]{5.8,4,1.2,0.2},
			new double[]{5.7,4.4,1.5,0.4},
			new double[]{5.4,3.9,1.3,0.4},
			new double[]{5.1,3.5,1.4,0.3},
			new double[]{5.7,3.8,1.7,0.3},
			new double[]{5.1,3.8,1.5,0.3},
			new double[]{5.4,3.4,1.7,0.2},
			new double[]{5.1,3.7,1.5,0.4},
			new double[]{4.6,3.6,1,0.2},
			new double[]{5.1,3.3,1.7,0.5},
			new double[]{4.8,3.4,1.9,0.2},
			new double[]{5,3,1.6,0.2},
			new double[]{5,3.4,1.6,0.4},
			new double[]{5.2,3.5,1.5,0.2},
			new double[]{5.2,3.4,1.4,0.2},
			new double[]{4.7,3.2,1.6,0.2},
			new double[]{4.8,3.1,1.6,0.2},
			new double[]{5.4,3.4,1.5,0.4},
			new double[]{5.2,4.1,1.5,0.1},
			new double[]{5.5,4.2,1.4,0.2},
			new double[]{4.9,3.1,1.5,0.1},
			new double[]{5,3.2,1.2,0.2},
			new double[]{5.5,3.5,1.3,0.2},
			new double[]{4.9,3.1,1.5,0.1},
			new double[]{4.4,3,1.3,0.2},
			new double[]{5.1,3.4,1.5,0.2},
			new double[]{5,3.5,1.3,0.3},
			new double[]{4.5,2.3,1.3,0.3},
			new double[]{4.4,3.2,1.3,0.2},
			new double[]{5,3.5,1.6,0.6},
			new double[]{5.1,3.8,1.9,0.4},
			new double[]{4.8,3,1.4,0.3},
			new double[]{5.1,3.8,1.6,0.2},
			new double[]{4.6,3.2,1.4,0.2},
			new double[]{5.3,3.7,1.5,0.2},
			new double[]{5,3.3,1.4,0.2},
			new double[]{7,3.2,4.7,1.4},
			new double[]{6.4,3.2,4.5,1.5},
			new double[]{6.9,3.1,4.9,1.5},
			new double[]{5.5,2.3,4,1.3},
			new double[]{6.5,2.8,4.6,1.5},
			new double[]{5.7,2.8,4.5,1.3},
			new double[]{6.3,3.3,4.7,1.6},
			new double[]{4.9,2.4,3.3,1},
			new double[]{6.6,2.9,4.6,1.3},
			new double[]{5.2,2.7,3.9,1.4},
			new double[]{5,2,3.5,1},
			new double[]{5.9,3,4.2,1.5},
			new double[]{6,2.2,4,1},
			new double[]{6.1,2.9,4.7,1.4},
			new double[]{5.6,2.9,3.6,1.3},
			new double[]{6.7,3.1,4.4,1.4},
			new double[]{5.6,3,4.5,1.5},
			new double[]{5.8,2.7,4.1,1},
			new double[]{6.2,2.2,4.5,1.5},
			new double[]{5.6,2.5,3.9,1.1},
			new double[]{5.9,3.2,4.8,1.8},
			new double[]{6.1,2.8,4,1.3},
			new double[]{6.3,2.5,4.9,1.5},
			new double[]{6.1,2.8,4.7,1.2},
			new double[]{6.4,2.9,4.3,1.3},
			new double[]{6.6,3,4.4,1.4},
			new double[]{6.8,2.8,4.8,1.4},
			new double[]{6.7,3,5,1.7},
			new double[]{6,2.9,4.5,1.5},
			new double[]{5.7,2.6,3.5,1},
			new double[]{5.5,2.4,3.8,1.1},
			new double[]{5.5,2.4,3.7,1},
			new double[]{5.8,2.7,3.9,1.2},
			new double[]{6,2.7,5.1,1.6},
			new double[]{5.4,3,4.5,1.5},
			new double[]{6,3.4,4.5,1.6},
			new double[]{6.7,3.1,4.7,1.5},
			new double[]{6.3,2.3,4.4,1.3},
			new double[]{5.6,3,4.1,1.3},
			new double[]{5.5,2.5,4,1.3},
			new double[]{5.5,2.6,4.4,1.2},
			new double[]{6.1,3,4.6,1.4},
			new double[]{5.8,2.6,4,1.2},
			new double[]{5,2.3,3.3,1},
			new double[]{5.6,2.7,4.2,1.3},
			new double[]{5.7,3,4.2,1.2},
			new double[]{5.7,2.9,4.2,1.3},
			new double[]{6.2,2.9,4.3,1.3},
			new double[]{5.1,2.5,3,1.1},
			new double[]{5.7,2.8,4.1,1.3},
			new double[]{6.3,3.3,6,2.5},
			new double[]{5.8,2.7,5.1,1.9},
			new double[]{7.1,3,5.9,2.1},
			new double[]{6.3,2.9,5.6,1.8},
			new double[]{6.5,3,5.8,2.2},
			new double[]{7.6,3,6.6,2.1},
			new double[]{4.9,2.5,4.5,1.7},
			new double[]{7.3,2.9,6.3,1.8},
			new double[]{6.7,2.5,5.8,1.8},
			new double[]{7.2,3.6,6.1,2.5},
			new double[]{6.5,3.2,5.1,2},
			new double[]{6.4,2.7,5.3,1.9},
			new double[]{6.8,3,5.5,2.1},
			new double[]{5.7,2.5,5,2},
			new double[]{5.8,2.8,5.1,2.4},
			new double[]{6.4,3.2,5.3,2.3},
			new double[]{6.5,3,5.5,1.8},
			new double[]{7.7,3.8,6.7,2.2},
			new double[]{7.7,2.6,6.9,2.3},
			new double[]{6,2.2,5,1.5},
			new double[]{6.9,3.2,5.7,2.3},
			new double[]{5.6,2.8,4.9,2},
			new double[]{7.7,2.8,6.7,2},
			new double[]{6.3,2.7,4.9,1.8},
			new double[]{6.7,3.3,5.7,2.1},
			new double[]{7.2,3.2,6,1.8},
			new double[]{6.2,2.8,4.8,1.8},
			new double[]{6.1,3,4.9,1.8},
			new double[]{6.4,2.8,5.6,2.1},
			new double[]{7.2,3,5.8,1.6},
			new double[]{7.4,2.8,6.1,1.9},
			new double[]{7.9,3.8,6.4,2},
			new double[]{6.4,2.8,5.6,2.2},
			new double[]{6.3,2.8,5.1,1.5},
			new double[]{6.1,2.6,5.6,1.4},
			new double[]{7.7,3,6.1,2.3},
			new double[]{6.3,3.4,5.6,2.4},
			new double[]{6.4,3.1,5.5,1.8},
			new double[]{6,3,4.8,1.8},
			new double[]{6.9,3.1,5.4,2.1},
			new double[]{6.7,3.1,5.6,2.4},
			new double[]{6.9,3.1,5.1,2.3},
			new double[]{5.8,2.7,5.1,1.9},
			new double[]{6.8,3.2,5.9,2.3},
			new double[]{6.7,3.3,5.7,2.5},
			new double[]{6.7,3,5.2,2.3},
			new double[]{6.3,2.5,5,1.9},
			new double[]{6.5,3,5.2,2},
			new double[]{6.2,3.4,5.4,2.3},
			new double[]{5.9,3,5.1,1.8}
		};
	}
	
	final static int[] getTarget() {
		return new int[]{
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
			2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
		};
	}
	
	final static String[] getHeaders() {
		return new String[]{"Sepal Length","Sepal Width","Petal Length","Petal Width"};
	}
}
