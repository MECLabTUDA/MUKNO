/**
 * XPIWIT - An XML Pipeline Wrapper for the Insight Toolkit.
 * Copyright (C) 2015 A. Bartschat, E. Hübner, M. Reischl, R. Mikut and J. Stegmaier
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please refer to the documentation for more information about the software
 * as well as for installation instructions.
 *
 * If you use this application for your scientific work, please cite the following publication
 *
 * A. Bartschat, E. Hübner, M. Reischl, R. Mikut, J. Stegmaier,
 * XPIWIT - An XML Pipeline Wrapper for the Insight Toolkit. 2015.
 *
 */

#ifndef ALGABSTRACTINPUT_H
#define ALGABSTRACTINPUT_H

// qt header
#include <QtCore/QString>


/**
	* @desc abstract class to represent an input of an image filter
	*/
class AlgAbstractInput
{
public:

	/**
		* The constructor.
		*/
	AlgAbstractInput();

	/**
		* Constructor with initialization.
		* @param idRef the id for identifying the respective filter.
		* @param numberRef the number of the input to use.
		* @param dataType the data type of the input.
		*/
	AlgAbstractInput(QString idRef, int numberRef, int dataType)
	{
		mIdRef = idRef;
		mNumberRef = numberRef;
		mDataType = dataType;
	}

	// Member variables
	QString mIdRef;
	int mNumberRef;
	int mDataType;
};

#endif // ALGABSTRACTINPUT_H
