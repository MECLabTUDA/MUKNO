#pragma once

#include <QApplication.h>

#include <exception>

QT_BEGIN_NAMESPACE

class SafeApplication : public QApplication
{
  public:
    SafeApplication(int &argc, char **argv, int flag = ApplicationFlags);
    virtual ~SafeApplication();

  public:
    void setupConnections();

  public:
    bool notify(QObject *receiver_, QEvent *event_);
};

QT_END_NAMESPACE