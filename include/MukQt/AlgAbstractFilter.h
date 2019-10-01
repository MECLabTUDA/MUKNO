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

#ifndef ALGABSTRACTFILTER_H
#define ALGABSTRACTFILTER_H

// namespace header
#include "AlgAbstractInput.h"

// qt header
#include <QtCore/QPair>
#include <QtCore/QList>
#include <QtCore/QString>
#include <QVector>
#include <qstring.h>

class AlgAbstractFilter
{
public:

	/**
	* the default constructor.
	*/
	AlgAbstractFilter() { mIsReader = false; }

	/**
	* constructor with initialization of the filter
	* @param filter the filter to initialize the class members to.
	*/
	AlgAbstractFilter(AlgAbstractFilter& filter)
	{
		SetAbstractFilter(filter);
	}

	/**
	* function to set the abstract filter members.
	* @param filter the filter to initialize the class members to.
	*/
	void SetAbstractFilter(AlgAbstractFilter& filter)
	{
		mId = filter.GetId();                              // id of the filter from xml
		mName = filter.GetName();                                      // name of the filter
		mDescription = filter.GetDesription();								// description of filter
		mIsReader = filter.IsReader();                                     // is true for reader items with cmd_00 input
		mImageTypes = filter.GetTypes();							// list of types for templating
		mImageTypeIds = filter.GetTypeIds();							// list of types for templating
		mImageInputs = filter.GetImageInputs();               // image inputs
		mMetaInputs = filter.GetMetaInputs();                // meta inputs
		mParameter = filter.GetParameters();						// parameter of the filter
		mParameterDescriptions = filter.GetParameterDescriptions();				// description of the parameter at corresponding index
		mParameterTypes = filter.GetParameterTypes();			// Tyoes of the corresponding parameters
		mNumberImageIn = filter.GetNumberImageIn();
		mNumberImageOut = filter.GetNumberImageOut();
		mNumberMetaIn = filter.GetNumberMetaIn();
		mNumberMetaOut = filter.GetNumberMetaOut();

		mKeepOutput = filter.GetKeepOutput();
	}

	/**
	* Function to set the filter id string.
	* @param id the id of the filter, e.g., item_0001
	*/
	void SetId(QString id) { mId = id; }

	/**
	* Function to get the id of the filter.
	* @return id the id of the filter, e.g., item_0001
	*/
	QString GetId() { return mId; }
	int GetIdInt() { int foo; foo = mId.remove("item_").toInt(); return foo; }
	/**
	* Set the name of the filter
	* @param name the name of the filter.
	*/
	void SetName(QString name)
	{
		mName = name;
		if (name.toLower().compare("imagereader") == 0) mIsReader = true;
		if (name.toLower().compare("metareader") == 0) mIsReader = true;
	}

	/**
	* Get the name of the filter
	* @return the name of the filter.
	*/
	QString GetName() { return mName; }

	// 
	bool IsReader() { return mIsReader; }
	void SetIsReader(bool isReader) { mIsReader = isReader; }

	// types
	void SetType(QString type) { mImageTypes.append(type); }
	QStringList GetTypes() { return mImageTypes; }
	void SetTypeId(int typeId) { mImageTypeIds.append(typeId); }
	QList< int > GetTypeIds() { return mImageTypeIds; }

	// Image Input
	void SetImageInput(int inputId, QString idRef, int numberRef, int dataType) { mImageInputs[inputId].mIdRef = idRef; mImageInputs[inputId].mNumberRef = numberRef; mImageInputs[inputId].mDataType = dataType; mImageTypeIds[inputId] = dataType; }
	void SetImageInput(QString idRef, int numberRef, int dataType) { SetImageInput(AlgAbstractInput(idRef, numberRef, dataType)); }
	void SetImageInput(AlgAbstractInput input) { mImageInputs.append(input); SetTypeId(input.mDataType); }
	QList< AlgAbstractInput > GetImageInputs() { return mImageInputs; }

	// Meta Input
	void SetMetaInput(int inputId, QString idRef, int numberRef, int dataType) { mMetaInputs[inputId].mIdRef = idRef; mMetaInputs[inputId].mNumberRef = numberRef; mMetaInputs[inputId].mDataType = dataType; }
	void SetMetaInput(QString idRef, int numberRef, int dataType) { SetMetaInput(AlgAbstractInput(idRef, numberRef, dataType)); }
	void SetMetaInput(AlgAbstractInput input) { mMetaInputs.append(input); }
	QList< AlgAbstractInput > GetMetaInputs() { return mMetaInputs; }

	const QList< QString > GetRequiredIds()
	{
		QList< QString > ids;
		foreach(AlgAbstractInput input, mImageInputs) { ids.append(input.mIdRef); }
		foreach(AlgAbstractInput input, mMetaInputs) { ids.append(input.mIdRef); }
		return ids;
	}

	// Parameter
	void SetParameter(QString key, QString value)
	{
		QVector<QString> parameter;
		parameter.append(key);
		parameter.append(value);
		SetParameter(parameter);
	}

	void SetParameter(QVector<QString> parameter) { mParameter.append(parameter); }
	void SetParameter(int index, QString value) { mParameter[index][1] = value; }
	int GetNumberParameter() { return mParameter.length(); }
	QVector<QString> GetParameter(int number) { return mParameter.at(number); }
	QList< QVector<QString> > GetParameters() { return mParameter; }

	void SetKeepOutput(int num) { mKeepOutput.append(num); }
	int GetKeepOutput(int num) { return mKeepOutput.count(num); }
	QList<int> GetKeepOutput() { return mKeepOutput; }

	// Descriptions ATTENTION: Parameter descriptions don't automatically stay in sync with parameters! For now you have to manage that yourself!
	void SetDescription(QString des) { mDescription = des; }
	QString GetDesription() { return mDescription; }
	void SetParDescriptions(QList<QString> des) { mParameterDescriptions = des; }
	void AppendParDescription(QString des) { mParameterDescriptions.append(des); }
	void AppendParType(int type) { mParameterTypes.append(type); }
	QList<int> GetParameterTypes() { return mParameterTypes; }
	QList<QString> GetParameterDescriptions() { return mParameterDescriptions; }

	// In/out numbers
	int GetNumberImageIn() { return mNumberImageIn; }
	int GetNumberImageOut() { return mNumberImageOut; }
	int GetNumberMetaIn() { return mNumberMetaIn; }
	int GetNumberMetaOut() { return mNumberMetaOut; }
	void SetNumberImageIn(int number) { mNumberImageIn = number; }
	void SetNumberImageOut(int number) { mNumberImageOut = number; }
	void SetNumberMetaIn(int number) { mNumberMetaIn = number; }
	void SetNumberMetaOut(int number) { mNumberMetaOut = number; }

private:
	QString mId;                                // id of the filter from xml
	QString mName;                              // name of the filter
	QString mDescription;						// description of filter
	bool mIsReader;                             // is true for reader items with cmd_00 input
	QStringList mImageTypes;					// list of types for templating
	QList< int > mImageTypeIds;					// list of types for templating
	QList< AlgAbstractInput > mImageInputs;        // image inputs
	QList< AlgAbstractInput > mMetaInputs;         // meta inputs
	QList< QVector<QString> > mParameter;		// parameter of the filter
	QList<QString> mParameterDescriptions;		// description of the parameter at corresponding index
	QList<int> mParameterTypes;					// Type of corresponding parameter, 0=string, 1=double, 2=int, 3=bool
	int mNumberImageIn;
	int mNumberImageOut;
	int mNumberMetaIn;
	int mNumberMetaOut;

	QList< int > mKeepOutput;
};
#endif // ALGABSTRACTFILTER_H
