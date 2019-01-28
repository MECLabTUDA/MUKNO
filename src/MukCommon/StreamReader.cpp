#include "private/muk.pch"
#include "StreamReader.h"
#include "MukException.h"

#include <sstream>

namespace gris
{
namespace muk
{
  /**
  */
  StreamReader::StreamReader(std::istream& mStreams)
    : mStream(mStreams)
  {
    //mCurToken = ST;
    mInt = 0;
    mDouble = 0.0;
  }

  /**
  */
  StreamReader::Token StreamReader::NextToken()
  {
    if (mStream.eof())
      return mCurToken = END;

    char c = END;
    try 
    {
      mStream >> c;
    } 
    catch (std::istream::failure&) 
    {
      if (mStream.eof())
        return mCurToken = END;
      else 
        return mCurToken = ERR;
    }

    switch(c)
    {
      case 0: 
        return mCurToken = END;
      case '(': case ')': case '{': case '}': case '[': case ']':
      case ':': case ';':
      case '=':
      case ',': case '.':
      case '°': case '*':
        return mCurToken=Token(c);
      case '"':
        if (ReadQuotedString(c)) 
          return mCurToken = STRING;
        else
          return mCurToken = ERR;
        break;
      case '0': case '1': case '2': case '3': case '4': 
      case '5': case '6': case '7': case '8': case '9':
        return mCurToken = ReadNumber(c);
      case '-': case '+':
        return mCurToken = ReadNumberSigned(c);
      default:
      {
        if (isalpha(c) || c == '_') 
        {
          ReadString(c);		
          return mCurToken = STRING;
        }
        if (mStream.eof()) 
        {
          return mCurToken = END;
        }
        else 
        {
          mChar = c;
          return mCurToken = CHAR;
        }
      }
    }
  }

  /**
  */
  std::string StreamReader::ReadLine()
  {	
    /*unsigned char c;	
    do 
    {
      mStream >> c;
    }
    while (c == ' ');
    mStream.putback(c);*/
    std::string str;
    std::getline(mStream, str);
    NextToken();
    return str;
  }

  /**
  */
  void StreamReader::ReadUntilEndOfLine()
  {
    char c = 0;
    while (!mStream.eof() && c != '\n') 
    {
      mStream.get(c);
    }
    NextToken();
  }

  /**
  */
  void StreamReader::ReadString(char firstChar)	
  {
    std::ostringstream ost;
    char c = firstChar;
    while (isalpha(c) || isdigit(c) || c == '_') 
    {
      ost << c;
      if (!mStream.get(c)) 
      {
        mString = ost.str();
        return;		// End of stream 
      }
    }
    mString = ost.str();
    // Put last character read from the stream which is not a letter back
    mStream.putback(c);
  }

  /**
  */
  bool StreamReader::ReadQuotedString(char firstChar)
  {
    std::ostringstream ost;
    char c;
    while (true) 
    {
      if (!mStream.get(c)) 
      {
        mString = ost.str();
        return false;		// End of stream, but we never saw the ending '"'
      }
      if (c == '"')
        break;
      else
        ost << c;
    }
    mString = ost.str();
    return true;
  }

  /**
  */
  StreamReader::Token	StreamReader::ReadNumber(char firstDigit)
  {
    // Quick an dirty (and inefficient)
    mStream.putback(firstDigit);
    double dValue;
    mStream >> dValue;
    int iValue = static_cast<int>(dValue);
    if (dValue - iValue == 0.0) 
    {
      mInt = iValue;
      return INUMBER;
    } 
    else 
    {
      mDouble = dValue;
      return FNUMBER;
    }
  }

  /**
  */
  StreamReader::Token StreamReader::ReadNumberSigned(char sign)
  {
    char c = 0;
    mStream >> c;
    if (c >= '0' && c <= '9') 
    {
      Token t = ReadNumber(c);
      if (sign == '-') 
      {
        if (t == INUMBER) 
          mInt = -mInt;
        else if (t == FNUMBER) 
          mDouble = -mDouble;
      }
      return t;
    } 
    else 
    {
      mStream.putback(c);
      return Token(sign);
    }
  }

  /**
  */
  void StreamReader::TokenAssertionFailed(Token t) const
  {
    switch(t) 
    {
      case STRING:
        throw MUK_EXCEPTION_SIMPLE("Syntax error: String expected");
      case INUMBER:
        throw MUK_EXCEPTION_SIMPLE("Syntax error: Integral number expected");
      case FNUMBER:
        throw MUK_EXCEPTION_SIMPLE("Syntax error: Floating point number expected");
      default:
        throw MUK_EXCEPTION_SIMPLE("Syntax error: Different token expected");
    }
  }

  /**
  */
  void StreamReader::WrongStringLiteral(const std::string& expected) const
  {
    std::stringstream stream;
    stream << "Syntax error: Expected '" << expected << "', found '" << mString << "'";
    throw MUK_EXCEPTION_SIMPLE(stream.str().c_str());
  }

  /**
  */
  void StreamReader::ExpectStringValue(const std::string& expectedValue) 
  {
    AssertToken(STRING);
    if (mString != expectedValue)
      WrongStringLiteral(expectedValue);	
    NextToken();
  }

  /*
  */
  void StreamReader::ExpectStringLC(const std::string& expectedValue)
  {
    std::string strLower = mString;
    std::transform(strLower.begin(), strLower.end(), strLower.begin(), tolower);
    AssertToken(STRING);
    if (strLower != expectedValue)
      WrongStringLiteral(expectedValue);	
    NextToken();
  }

  /**
  */
  double StreamReader::ReadDoubleStrict()
  {
    AssertToken(FNUMBER);
    double dRet = GetDoubleValue();
    NextToken();
    return dRet;
  }

  /**
  */
  double StreamReader::ReadDouble() 
  {
    double dRet = 0;
    if (mCurToken == FNUMBER) 
    {
      dRet = GetDoubleValue();	
      NextToken();				
    }
    else if (mCurToken == INUMBER) 
    {
      dRet = GetIntValue();
      NextToken();
    } 
    else 
    {
      TokenAssertionFailed(FNUMBER);
    }
    return dRet;
  }

  /**
  */
  int StreamReader::ReadInt() 
  {
    AssertToken(INUMBER);
    int iRet = GetIntValue();
    NextToken();
    return iRet;				
  }

  /**
  */
  void StreamReader::ReadString(std::string& str)
  {
    AssertToken(STRING);
    str = GetStringValue();
    NextToken();
  }

  /**
  */
  void StreamReader::ReadToken(Token t) 
  {
    AssertToken(t);
    NextToken();
  }
}
}