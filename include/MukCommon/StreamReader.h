#pragma once
#include "muk_common_api.h"

#include <algorithm>
#include <string>

namespace gris
{
  namespace muk
  {
    /** \brief Converts an input stream into a sequence of tokens
    */
    class MUK_COMMON_API StreamReader 
    {
      public:
        explicit StreamReader(std::istream& inps);
        
      public: 
        // Possible Tokens
        enum Token 
        {
          BEGIN, END, ERR,
          STRING, INUMBER, FNUMBER, CHAR,
          LP	= '(', RP = ')',
          LCB = '{', RCB = '}',
          LSB = '[', RSB = ']',
          COLON = ':', SEMICOLON = ';',
          EQUAL = '=', COMMA = ',',
          POINT = '.', DEGREE= '°',
          PLUS = '+', MINUS = '-',
          ASTERISK = '*'
        };

      public:
        Token               CurrentToken()   const { return mCurToken;	}
        int                 GetIntValue()    const { return mInt; }
        double              GetDoubleValue() const { return mDouble; }
        char                GetCharValue()   const { return mChar; }
        const std::string&  GetStringValue() const { return mString; }

        void                AssertToken(Token token) const { if (token != mCurToken) TokenAssertionFailed(token); }

      public:
        Token       NextToken();
        double      ReadDoubleStrict();
        double      ReadDouble();
        int         ReadInt();
        void        ReadString(std::string& str);
        void        ReadToken(Token t);
        std::string ReadLine();
        void        ReadUntilEndOfLine();
        void        Reset() {	mCurToken = BEGIN; }
        
        void        ExpectStringValue(const std::string& expectedValue);
        void        ExpectStringLC(const std::string& expectedValue);
        

      private:
        void  TokenAssertionFailed(Token token) const; 
        Token ReadNumber(char firstDigit);
        Token ReadNumberSigned(char sign);
        void  ReadString (char firstChar);
        bool  ReadQuotedString(char firstChar);
        void  WrongStringLiteral(const std::string& expected) const; 

    private:
        std::istream&   mStream;
        Token			      mCurToken;
        int				      mInt;
        double			    mDouble;
        char			      mChar;
        std::string		  mString;
        /*unsigned int	  mCurLine;
        unsigned long	  mCurChar;
        unsigned int	  mCurColumn;*/
    };
  }
}
