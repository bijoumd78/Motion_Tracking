#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"
#include <QTimer>
#include <QTcpSocket>
#include <QNetworkSession>
#include <vector>
#include "Vector_3D.h"
#include <QFile>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setupRealtimeData(QCustomPlot *customPlot);

private slots:
    void realtimeDataSlot_X();
    void realtimeDataSlot_Y();
    void realtimeDataSlot_Z();
    void realtimeDataSlot_XROT();
    void realtimeDataSlot_YROT();
    void realtimeDataSlot_ZROT();
    // Network functions
    void readSensorData();
    void displayError(QAbstractSocket::SocketError socketError);
    void sessionOpened();
    void sendRequest();

protected:
    void closeEvent(QCloseEvent*);


private:
    // Utility function
    void delay(int msec);

    // Variables
    bool isClosed;
    Ui::MainWindow *ui;
    QTimer dataTimer;
    // Network
    QTcpSocket *tcpSocket;
    quint16 blockSize;
    QNetworkSession *networkSession;
    int DataRate;

    // Received tracking data
    std::vector<double> coll_x, coll_y, coll_z;
    std::vector<Vector_3D<double>> coll_nvt, coll_tvt;
    double dXPos, dYPos, dZPos, dXRot, dYRot, dZRot;

    // Log file
    QFile *myFile;
    QTextStream myLog();
    QString Log_path;

};

#endif // MAINWINDOW_H
