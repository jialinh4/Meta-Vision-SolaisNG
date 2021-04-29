#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
//#include "AnnotatedMatViewer.h"
#include "TerminalSocket.h"
#include "Parameters.pb.h"

namespace Ui {
class MainWindow;
}

class QLabel;
class QTimer;

namespace meta {

class PhaseController;

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

private:
    Ui::MainWindow *ui;
    PhaseController* phases;
    QTimer *statsUpdateTimer;

    boost::asio::io_context ioContext;
    QTimer *ioTimer;
    TerminalSocketClient socket;

    // Reuse message objects: developers.google.com/protocol-buffers/docs/cpptutorial#optimization-tips
    package::ParamSet paramsMessage;
    package::Result resultMessage;

    void handleClientDisconnection(TerminalSocketClient *client);

    void handleRecvBytes(std::string_view name, const uint8_t *buf, size_t size);

    void handleRecvSingleString(std::string_view name, std::string_view s);

    void handleRecvSingleInt(std::string_view name, int val);

    void handleRecvListOfStrings(std::string_view name, const vector<const char *> &list);

    static QString bytesToDateRate(unsigned bytes);

private slots:

    void connectToServer();

    void updateStats();

    void performIO();

};

}

#endif // MAINWINDOW_H
