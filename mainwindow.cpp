#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtNetwork>
#include <Windows.h>
#include <array>

#define PI 3.141592653589793

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    isClosed(false),
    networkSession(0),
    blockSize(0),
    DataRate(20), // data rate
    dXPos(0.0),
    dYPos(0.0),
    dZPos(0.0),
    dXRot(0.0),
    dYRot(0.0),
    dZRot(0.0),
    Log_path( QDir::currentPath())

{
    ui->setupUi(this);
    setWindowTitle("Live Motion Detection");
    ui->statusBar->hide(); // hide menubar

    // Network Connect to EndoScout server
    // TCP/IP socket
    tcpSocket = new QTcpSocket(this);
    if (tcpSocket->state() != QTcpSocket::ConnectedState)
    {
        tcpSocket->connectToHost("192.168.2.5", 20248); // EndoScout parameters
    }

    connect(tcpSocket, SIGNAL(connected()), this, SLOT(sendRequest()));

    connect(tcpSocket, SIGNAL(readyRead()), this, SLOT(readSensorData()));

    connect(tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(displayError(QAbstractSocket::SocketError)));

    QNetworkConfigurationManager manager;
    if (manager.capabilities() & QNetworkConfigurationManager::NetworkSessionRequired) {
        // Get saved network configuration
        QSettings settings(QSettings::UserScope, QLatin1String("QtProject"));
        settings.beginGroup(QLatin1String("QtNetwork"));
        const QString id = settings.value(QLatin1String("DefaultNetworkConfiguration")).toString();
        settings.endGroup();

        // If the saved network configuration is not currently discovered use the system default
        QNetworkConfiguration config = manager.configurationFromIdentifier(id);
        if ((config.state() & QNetworkConfiguration::Discovered) !=
                QNetworkConfiguration::Discovered) {
            config = manager.defaultConfiguration();
        }

        networkSession = new QNetworkSession(config, this);
        connect(networkSession, SIGNAL(opened()), this, SLOT(sessionOpened()));
        networkSession->open();
    }

    // give the axes some labels:
    ui->widget->yAxis->setLabel("X_Position [mm]");
    ui->widget_2->yAxis->setLabel("Y_Position [mm]");
    ui->widget_3->yAxis->setLabel("Z_Position [mm]");
    ui->widget_4->yAxis->setLabel("X_Rotation [deg]");
    ui->widget_5->yAxis->setLabel("Y_Rotation [deg]");
    ui->widget_6->yAxis->setLabel("Z_Rotation [deg]");
    //ui->widget_5->xAxis->setLabel("Time");

    setupRealtimeData(ui->widget);
    ui->widget->replot();
    setupRealtimeData(ui->widget_2);
    ui->widget_2->replot();
    setupRealtimeData(ui->widget_3);
    ui->widget_3->replot();
    setupRealtimeData(ui->widget_4);
    ui->widget_4->replot();
    setupRealtimeData(ui->widget_5);
    ui->widget_5->replot();
    setupRealtimeData(ui->widget_6);
    ui->widget_6->replot();

    // Logger
    auto myString =  QTime::currentTime().toString();
    QString hours   = myString.mid(0, 2);
    QString minutes = myString.mid(3, 2);
    QString seconds = myString.mid(6, 2);
    const QString LogFileName = Log_path + QString("/Logger/Log_") + QDate::currentDate().toString("dd.MM.yyyy") + QString("_") +
            hours + QString(".") + minutes + QString(".") + seconds + QString(".txt");

    myFile = new QFile(LogFileName);
    myFile->open(QIODevice::WriteOnly|QIODevice::Text);

}

MainWindow::~MainWindow()
{
    //Close Log file
    myFile->close();
    // close the connection
    tcpSocket->close();
    //Flush the containers
    coll_x.clear();
    coll_y.clear();
    coll_z.clear();
    coll_nvt.clear();
    coll_tvt.clear();

    delete ui;
}

void MainWindow::readSensorData()
{
    // Received data parameters
    std::array<double, 3> XYZ, nvt, tvt;
    QString channel;
    QString StatusCode;


    while(!isClosed){

        if(tcpSocket->bytesAvailable()){
            //Get the data
            QString s = QString::fromLatin1((tcpSocket->readAll()).constData());

            // Parse the received string
            QTextStream(&s) >> channel >> XYZ[0] >> XYZ[1] >> XYZ[2] >> nvt[0] >>nvt[1]
                            >> nvt[2] >> tvt[0] >> tvt[1] >> tvt[2] >> StatusCode;

            // Check for good tracking status and store the data
            if(StatusCode == QString("1000")){

                coll_x.push_back(XYZ[0]);
                coll_y.push_back(XYZ[1]);
                coll_z.push_back(XYZ[2]);

                coll_nvt.push_back(Vector_3D<double>(nvt[0], nvt[1], nvt[2]));
                coll_tvt.push_back(Vector_3D<double>(tvt[0],  tvt[1], tvt[2]));

                Vector_3D<double> a(0,0,0), b(0,0,0), c(0,0,0);
                a = coll_nvt.front().crossproduct( coll_nvt.back() );
                b = coll_tvt.front().crossproduct( coll_tvt.back() );
                c = coll_nvt.front().crossproduct( coll_tvt.front() );


                // Compute the relative position
                dXPos = coll_x.front() - coll_x.back();
                dYPos = coll_y.front() - coll_y.back();
                dZPos = coll_z.front() - coll_z.back();


                // Compute the rotation angles (FYI: acos method is unstable)
                double angle_X = atan2( sqrt( pow( a.get_x(), 2 ) + pow( a.get_y(), 2 ) + pow( a.get_z(), 2 ) ),  coll_nvt.front().dotproduct( coll_nvt.back() ) );
                c.dotproduct( a ) < 0    ?  dXRot = (-180/PI) * angle_X  :  dXRot =  (180/PI) * angle_X; // Compute the sign of the angle

                double angle_Y = atan2( sqrt( pow( b.get_x(), 2 ) + pow( b.get_y(), 2 ) + pow( b.get_z(), 2 ) ),  coll_tvt.front().dotproduct( coll_tvt.back() ) );
                coll_nvt.front().dotproduct( b ) < 0  ?  dYRot = (-180/PI) * angle_Y  :  dYRot =  (180/PI) * angle_Y ; // Compute the sign of the angle

                double angle_Z = atan2( sqrt( pow( a.get_x(), 2 ) + pow( a.get_y(), 2 ) + pow( a.get_z(), 2 ) ),  coll_nvt.front().dotproduct( coll_nvt.back() ) );
                coll_tvt.front().dotproduct( a ) < 0  ?  dZRot = (-180/PI) * angle_Z  :  dZRot =  (180/PI) * angle_Z ; // Compute the sign of the angle
            }
            else{
                // Default values
                dXPos = 0.0;
                dYPos = 0.0;
                dZPos = 0.0;
                dXRot = 0.0;
                dYRot = 0.0;
                dZRot = 0.0;
            }
        }
        // Wait for a second before sending another request
        // This function doesn't freeze the GUI
        delay(100); // unit msec.
        emit sendRequest();
    }

    // close the connection
    tcpSocket->close();
}

void MainWindow::delay(int msec)
{
    //QTime dieTime= QTime::currentTime().addSecs(second);
    QTime dieTime= QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::displayError(QAbstractSocket::SocketError socketError)
{
    switch (socketError) {
    case QAbstractSocket::RemoteHostClosedError:
        break;
    case QAbstractSocket::HostNotFoundError:
        QMessageBox::information(this, tr("Live Motion Detection"),
                                 tr("The host was not found. Please check the "
                                    "host name and port settings. "
                                    "Please restart the program"));
        break;
    case QAbstractSocket::ConnectionRefusedError:
        QMessageBox::information(this, tr("Live Motion Detection"),
                                 tr("The connection was refused by the peer. "
                                    "Make sure the EndoScout server is running, "
                                    "and check that the host name and port "
                                    "settings are correct. "
                                    "Please restart the program"));
        break;
    default:
        QMessageBox::information(this, tr("Live Motion Detection"),
                                 tr("The following error occurred: %1.")
                                 .arg(tcpSocket->errorString()));
    }
}

void MainWindow::sessionOpened()
{
    // Save the used configuration
    QNetworkConfiguration config = networkSession->configuration();
    QString id;
    if (config.type() == QNetworkConfiguration::UserChoice)
        id = networkSession->sessionProperty(QLatin1String("UserChoiceConfiguration")).toString();
    else
        id = config.identifier();

    QSettings settings(QSettings::UserScope, QLatin1String("QtProject"));
    settings.beginGroup(QLatin1String("QtNetwork"));
    settings.setValue(QLatin1String("DefaultNetworkConfiguration"), id);
    settings.endGroup();
}

void MainWindow::sendRequest()
{
    // Request string
    const char request[]  = "DATA 1";
    tcpSocket->write(request, strlen(request));
    tcpSocket->waitForBytesWritten(1000);
}

void MainWindow::closeEvent(QCloseEvent *)
{
    isClosed = true;
}


void MainWindow::setupRealtimeData(QCustomPlot *customPlot)
{
    // include this section to fully disable antialiasing for higher performance:
    customPlot->setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    customPlot->xAxis->setTickLabelFont(font);
    customPlot->yAxis->setTickLabelFont(font);
    customPlot->legend->setFont(font);

    customPlot->addGraph(); // blue line
    customPlot->graph(0)->setPen(QPen(Qt::green, 2));
    //customPlot->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
    customPlot->graph(0)->setAntialiasedFill(false);

    customPlot->addGraph(); // blue dot
    customPlot->graph(1)->setPen(QPen(Qt::green));
    customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    customPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);

    customPlot->xAxis->setBasePen(QPen(Qt::white, 1));
    customPlot->yAxis->setBasePen(QPen(Qt::white, 1));
    customPlot->xAxis->setTickPen(QPen(Qt::white, 1));
    customPlot->yAxis->setTickPen(QPen(Qt::white, 1));
    customPlot->xAxis->setSubTickPen(QPen(Qt::white, 1));
    customPlot->yAxis->setSubTickPen(QPen(Qt::white, 1));
    customPlot->xAxis->setTickLabelColor(Qt::white);
    customPlot->yAxis->setTickLabelColor(Qt::white);
    //customPlot->yAxis->setRange(-50.0, 50.0);
    customPlot->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    customPlot->yAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    customPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    customPlot->xAxis->setDateTimeFormat("hh:mm:ss");
    customPlot->xAxis->setAutoTickStep(false);
    customPlot->xAxis->setTickStep(2);
    customPlot->yAxis->setLabelColor(Qt::white);
    customPlot->setBackground(Qt::black);
    //customPlot->yAxis->grid()->setVisible(false);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot_X()));
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot_Y()));
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot_Z()));
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot_XROT()));
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot_YROT()));
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot_ZROT()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void MainWindow::realtimeDataSlot_X()
{
    // calculate two new data points:
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    static double lastPointKey = 0;
    if (key-lastPointKey > 0.01) // at most add point every 10 ms
    {
        //dXPos = qSin(key);
        double value0 = dXPos;  /*qSin(key);*/

        // Alarm
        /*if (value0 > 5 || value0 < -5)
            Beep(523,500); // 523 hertz (C5) for 500 milliseconds*/

        // add data to lines:
        ui->widget->graph(0)->addData(key, value0);
        // set data of dots:
        ui->widget->graph(1)->clearData();
        ui->widget->graph(1)->addData(key, value0);
        // remove data of lines that's outside visible range:
        ui->widget->graph(0)->removeDataBefore(key - DataRate);
        // rescale value (vertical) axis to fit the current data:
        ui->widget->graph(0)->rescaleValueAxis();
        ui->widget->graph(1)->rescaleValueAxis(true);
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of DataRate):
    ui->widget->xAxis->setRange(key+0.25, DataRate, Qt::AlignRight);
    ui->widget->replot();

    // Start logging Sensor data
    QTextStream(myFile) << QString("LMD:\t") << QString::number(dXPos, 'f', 6) + QString("\t") + QString::number(dYPos, 'f', 6) + QString("\t") +QString::number(dZPos, 'f', 6)
                        << QString("\t") + QString::number(dXRot, 'f', 6) + QString("\t") + QString::number(dYRot, 'f', 6) + QString("\t") + QString::number(dZRot, 'f', 6) +
                           QString("\t") + QTime::currentTime().toString() << endl;

}

void MainWindow::realtimeDataSlot_Y()
{
    // calculate two new data points:
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    static double lastPointKey = 0;
    if (key-lastPointKey > 0.01) // at most add point every 10 ms
    {
        //dYPos = qCos(key);
        double value0 = dYPos;

        // Alarm
        /* if (value0 > 5 || value0 < -5)
            Beep(523,500); // 523 hertz (C5) for 500 milliseconds*/

        // add data to lines:
        ui->widget_2->graph(0)->addData(key, value0);
        // set data of dots:
        ui->widget_2->graph(1)->clearData();
        ui->widget_2->graph(1)->addData(key, value0);
        // remove data of lines that's outside visible range:
        ui->widget_2->graph(0)->removeDataBefore(key - DataRate);
        // rescale value (vertical) axis to fit the current data:
        ui->widget_2->graph(0)->rescaleValueAxis();
        ui->widget_2->graph(1)->rescaleValueAxis(true);
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of DataRate):
    ui->widget_2->xAxis->setRange(key+0.25, DataRate, Qt::AlignRight);
    ui->widget_2->replot();
}

void MainWindow::realtimeDataSlot_Z()
{
    // calculate two new data points:
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    static double lastPointKey = 0;
    if (key-lastPointKey > 0.01) // at most add point every 10 ms
    {
        // dZPos = qTan(key);
        double value0 = dZPos;

        // Alarm
        /* if (value0 > 5 || value0 < -5)
            Beep(523,500); // 523 hertz (C5) for 500 milliseconds*/

        // add data to lines:
        ui->widget_3->graph(0)->addData(key, value0);
        // set data of dots:
        ui->widget_3->graph(1)->clearData();
        ui->widget_3->graph(1)->addData(key, value0);
        // remove data of lines that's outside visible range:
        ui->widget_3->graph(0)->removeDataBefore(key - DataRate);
        // rescale value (vertical) axis to fit the current data:
        ui->widget_3->graph(0)->rescaleValueAxis();
        ui->widget_3->graph(1)->rescaleValueAxis(true);
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of DataRate):
    ui->widget_3->xAxis->setRange(key+0.25, DataRate, Qt::AlignRight);
    ui->widget_3->replot();
}

void MainWindow::realtimeDataSlot_XROT()
{
    // calculate two new data points:
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    static double lastPointKey = 0;
    if (key-lastPointKey > 0.01) // at most add point every 10 ms
    {
        //dXRot = qSin(key)+ 5*qCos(key);
        double value0 = dXRot;

        /* if (value0 > 5 || value0 < -5)
            Beep(523,500); // 523 hertz (C5) for 500 milliseconds*/

        // add data to lines:
        ui->widget_4->graph(0)->addData(key, value0);
        // set data of dots:
        ui->widget_4->graph(1)->clearData();
        ui->widget_4->graph(1)->addData(key, value0);
        // remove data of lines that's outside visible range:
        ui->widget_4->graph(0)->removeDataBefore(key - DataRate);
        // rescale value (vertical) axis to fit the current data:
        ui->widget_4->graph(0)->rescaleValueAxis();
        ui->widget_4->graph(1)->rescaleValueAxis(true);
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of DataRate):
    ui->widget_4->xAxis->setRange(key+0.25, DataRate, Qt::AlignRight);
    ui->widget_4->replot();
}

void MainWindow::realtimeDataSlot_YROT()
{
    // calculate two new data points:
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    static double lastPointKey = 0;
    if (key-lastPointKey > 0.01) // at most add point every 10 ms
    {
        //dYRot = qSin(key) - 5*qCos(key);
        double value0 = dYRot;

        // Alarm
        /* if (value0 > 5 || value0 < -5)
            Beep(523,500); // 523 hertz (C5) for 500 milliseconds*/

        // add data to lines:
        ui->widget_5->graph(0)->addData(key, value0);
        // set data of dots:
        ui->widget_5->graph(1)->clearData();
        ui->widget_5->graph(1)->addData(key, value0);
        // remove data of lines that's outside visible range:
        ui->widget_5->graph(0)->removeDataBefore(key - DataRate);
        // rescale value (vertical) axis to fit the current data:
        ui->widget_5->graph(0)->rescaleValueAxis();
        ui->widget_5->graph(1)->rescaleValueAxis(true);
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of DataRate):
    ui->widget_5->xAxis->setRange(key+0.25, DataRate, Qt::AlignRight);
    ui->widget_5->replot();
}

void MainWindow::realtimeDataSlot_ZROT()
{
    // calculate two new data points:
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    static double lastPointKey = 0;
    if (key-lastPointKey > 0.01) // at most add point every 10 ms
    {
        //dZRot = qSin(key) - 5*qCos(key);
        double value0 = dZRot;

        // Alarm
        /* if (value0 > 5 || value0 < -5)
            Beep(523,500); // 523 hertz (C5) for 500 milliseconds*/

        // add data to lines:
        ui->widget_6->graph(0)->addData(key, value0);
        // set data of dots:
        ui->widget_6->graph(1)->clearData();
        ui->widget_6->graph(1)->addData(key, value0);
        // remove data of lines that's outside visible range:
        ui->widget_6->graph(0)->removeDataBefore(key - DataRate);
        // rescale value (vertical) axis to fit the current data:
        ui->widget_6->graph(0)->rescaleValueAxis();
        ui->widget_6->graph(1)->rescaleValueAxis(true);
        lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of DataRate):
    ui->widget_6->xAxis->setRange(key+0.25, DataRate, Qt::AlignRight);
    ui->widget_6->replot();
}
