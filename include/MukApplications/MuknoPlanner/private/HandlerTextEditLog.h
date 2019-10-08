#pragma once

#include "MukAppModels/ApplicationModel.h"

#include "gstd/logger.h"

#include <QWidget>
#include <QTextEdit>
#include <QScrollBar>

namespace gris
{
  namespace muk
  {
    class QtLogWrapper : public QObject, LoggerCallback
    {
      Q_OBJECT

      public:
        QtLogWrapper()
        {
          GetGrisLogger().addCallback(this);
        }
        virtual ~QtLogWrapper() 
        {
          GetGrisLogger().removeCallback(this);
        }

      signals:
        void incomingMessage();

      public:
        virtual void stream(const char* str) 
        { 
          mBuffer << str << "\n";
          emit incomingMessage();
        }

        void getMessage(std::stringstream& ss)
        {
          ss << mBuffer.str();
          mBuffer.str( std::string() );
        }

      private:
        std::stringstream mBuffer;
    };

    /**
    */
    class HandlerTextEditLog : public QObject
    {
      Q_OBJECT

      public:
        struct Builder
        {
          QTextEdit* mpTextEdit;
          QtLogWrapper* mpWrapper;
        };

      public:
        explicit HandlerTextEditLog(const Builder& builder);
        ~HandlerTextEditLog();

      public slots:
        void incomingMessage()
        {
          std::stringstream ss;
          mpWrapper->getMessage(ss);          
          mpTextEdit->moveCursor(QTextCursor::End);
          mpTextEdit->insertPlainText(QString::fromLocal8Bit(ss.str().c_str(), ss.str().length()));
          mpTextEdit->moveCursor(QTextCursor::End);
          mpTextEdit->verticalScrollBar()->setValue(mpTextEdit->verticalScrollBar()->maximum());
        }


      private:
        void init();
        void setupConnections();
        
      private:
        QTextEdit*    mpTextEdit;
        QtLogWrapper* mpWrapper;
    };

  }
}
