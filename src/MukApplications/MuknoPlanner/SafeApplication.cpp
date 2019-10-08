#include "private/muk.pch"
#include "SafeApplication.h"

#include "MukCommon/MukException.h"

#include <QObject>
#include <QEvent>
#include <qmessagebox.h>

#include <boost/format.hpp>

#include <iostream>

QT_BEGIN_NAMESPACE

  SafeApplication::SafeApplication(int &argc, char **argv, int flag)
    : QApplication(argc, argv, flag)
  {
  }

  SafeApplication::~SafeApplication()
  {
  }

  void SafeApplication::setupConnections()
  {
    connect(this, &SafeApplication::lastWindowClosed, this, &SafeApplication::quit);
  }
    
  bool SafeApplication::notify(QObject *receiver_, QEvent *event_)
  {
    try
    {
      return QApplication::notify(receiver_, event_);
    }
    catch (const gris::muk::MukException& e)
    {
      // @Johannes maybe use ApplicationModel::handleException
      QMessageBox infobox;
      auto text = boost::format("Exception occured in:\n%s") % e.getFunction();
      infobox.setText(text.str().c_str());
      auto infoText = boost::format("%s\n\n%s") % e.getReason() % e.getInfo();
      LOG_LINE << "MukException\n" << text << "\n" << infoText;
      infobox.setInformativeText(infoText.str().c_str());
      infobox.setIcon(QMessageBox::Warning);
      infobox.setStandardButtons(QMessageBox::Ok);
      infobox.setWindowTitle("Gris-Exception");
      infobox.exec();
    }
    catch (const std::exception& e)
    {
      // @Johannes maybe use ApplicationModel::handleException
      QMessageBox infobox;
      infobox.setText("std::exception catched!");
      infobox.setInformativeText(e.what());      
      infobox.setIcon(QMessageBox::Warning);
      infobox.setStandardButtons(QMessageBox::Ok);
      infobox.setWindowTitle("std::exception");
      //infobox.setIconPixmap();
      infobox.exec();
    }

    return false;
  }

QT_END_NAMESPACE