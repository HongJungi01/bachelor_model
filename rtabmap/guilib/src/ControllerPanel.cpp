/*
 * ControllerPanel.cpp - see ControllerPanel.h.
 *
 * 1:1 behavioural port of default_Controller.py: 20 ms control tick with speed
 * ramping (1 s accel / 1 s decay), W/S/A/D + Space keyboard control, sliders,
 * hold-value spinboxes, E-stop, live E/P/C feedback and a command log.
 */
#include "ControllerPanel.h"

#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>

#ifdef RTABMAP_HAVE_QT_SERIALPORT
#include <QSerialPort>
#include <QSerialPortInfo>
#endif

namespace rtabmap {

ControllerPanel::ControllerPanel(QWidget * parent) :
	QWidget(parent)
{
	setFocusPolicy(Qt::StrongFocus);
	buildGui();
	refreshPorts();

	tickTimer_ = new QTimer(this);
	connect(tickTimer_, SIGNAL(timeout()), this, SLOT(tick()));
	tickTimer_->start(SEND_INTERVAL_MS);
}

ControllerPanel::~ControllerPanel()
{
	disconnectSerial();
}

// ===========================================================================
// GUI
// ===========================================================================
void ControllerPanel::buildGui()
{
	QVBoxLayout * root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(4);

	// ---- Serial connection ----
	QGroupBox * conn = new QGroupBox(tr("Serial connection"), this);
	QHBoxLayout * connL = new QHBoxLayout(conn);
	portCombo_ = new QComboBox(conn);
	portCombo_->setMinimumWidth(120);
	QPushButton * refreshBtn = new QPushButton(tr("Refresh"), conn);
	refreshBtn->setFocusPolicy(Qt::NoFocus);
	connectBtn_ = new QPushButton(tr("Connect"), conn);
	connectBtn_->setFocusPolicy(Qt::NoFocus);
	statusLabel_ = new QLabel(tr("disconnected"), conn);
	connL->addWidget(portCombo_);
	connL->addWidget(refreshBtn);
	connL->addWidget(connectBtn_);
	connL->addWidget(statusLabel_, 1);
	root->addWidget(conn);
	connect(refreshBtn, SIGNAL(clicked()), this, SLOT(refreshPorts()));
	connect(connectBtn_, SIGNAL(clicked()), this, SLOT(toggleConnect()));

	// ---- PID speed / steering ----
	QGroupBox * ctrl = new QGroupBox(tr("PID speed / steering"), this);
	QVBoxLayout * ctrlL = new QVBoxLayout(ctrl);

	QHBoxLayout * spd = new QHBoxLayout();
	spd->addWidget(new QLabel(tr("Speed:"), ctrl));
	speedSlider_ = new QSlider(Qt::Horizontal, ctrl);
	speedSlider_->setRange(-15, 15);
	speedSlider_->setFocusPolicy(Qt::NoFocus);
	speedValLabel_ = new QLabel("0", ctrl);
	speedValLabel_->setMinimumWidth(28);
	QPushButton * spdStop = new QPushButton(tr("Stop"), ctrl);
	spdStop->setFocusPolicy(Qt::NoFocus);
	spd->addWidget(speedSlider_, 1);
	spd->addWidget(speedValLabel_);
	spd->addWidget(spdStop);
	ctrlL->addLayout(spd);
	connect(speedSlider_, SIGNAL(valueChanged(int)), this, SLOT(onSpeedSlider(int)));
	connect(spdStop, SIGNAL(clicked()), this, SLOT(resetSpeed()));

	QHBoxLayout * ang = new QHBoxLayout();
	ang->addWidget(new QLabel(tr("Angle:"), ctrl));
	angleSlider_ = new QSlider(Qt::Horizontal, ctrl);
	angleSlider_->setRange(-20, 20);
	angleSlider_->setFocusPolicy(Qt::NoFocus);
	angleValLabel_ = new QLabel("0", ctrl);
	angleValLabel_->setMinimumWidth(28);
	QPushButton * angStop = new QPushButton(tr("Stop"), ctrl);
	angStop->setFocusPolicy(Qt::NoFocus);
	ang->addWidget(angleSlider_, 1);
	ang->addWidget(angleValLabel_);
	ang->addWidget(angStop);
	ctrlL->addLayout(ang);
	root->addWidget(ctrl);
	connect(angleSlider_, SIGNAL(valueChanged(int)), this, SLOT(onAngleSlider(int)));
	connect(angStop, SIGNAL(clicked()), this, SLOT(resetAngle()));

	// ---- Key hold values ----
	QGroupBox * setf = new QGroupBox(tr("Key hold values (W/S - A/D)"), this);
	QHBoxLayout * setL = new QHBoxLayout(setf);
	setL->addWidget(new QLabel(tr("Drive speed:"), setf));
	holdSpeedSpin_ = new QSpinBox(setf);
	holdSpeedSpin_->setRange(0, 15);
	holdSpeedSpin_->setValue(holdSpeed_);
	holdSpeedSpin_->setFocusPolicy(Qt::ClickFocus);
	setL->addWidget(holdSpeedSpin_);
	setL->addSpacing(12);
	setL->addWidget(new QLabel(tr("Steer angle:"), setf));
	holdAngleSpin_ = new QSpinBox(setf);
	holdAngleSpin_->setRange(0, 20);
	holdAngleSpin_->setValue(holdAngle_);
	holdAngleSpin_->setFocusPolicy(Qt::ClickFocus);
	setL->addWidget(holdAngleSpin_);
	setL->addStretch(1);
	root->addWidget(setf);
	connect(holdSpeedSpin_, SIGNAL(valueChanged(int)), this, SLOT(syncHoldValues()));
	connect(holdAngleSpin_, SIGNAL(valueChanged(int)), this, SLOT(syncHoldValues()));

	// ---- ALL STOP ----
	QPushButton * estop = new QPushButton(tr("ALL STOP"), this);
	estop->setFocusPolicy(Qt::NoFocus);
	estop->setMinimumHeight(44);
	estop->setStyleSheet("QPushButton{background:#ee3333;color:white;font-weight:bold;font-size:14px;}"
	                     "QPushButton:pressed{background:#cc0000;}");
	root->addWidget(estop);
	connect(estop, SIGNAL(clicked()), this, SLOT(allStop()));

	// ---- Feedback ----
	QGroupBox * fb = new QGroupBox(tr("Feedback (Arduino -> PC)"), this);
	QHBoxLayout * fbL = new QHBoxLayout(fb);
	encDeltaLabel_ = new QLabel("Enc Delta: ---", fb);
	potLabel_      = new QLabel("Pot (A0): ---", fb);
	encCumLabel_   = new QLabel("Enc Total: ---", fb);
	fbL->addWidget(encDeltaLabel_);
	fbL->addWidget(potLabel_);
	fbL->addWidget(encCumLabel_);
	root->addWidget(fb);

	// ---- Command preview ----
	QGroupBox * pv = new QGroupBox(tr("Sent command"), this);
	QHBoxLayout * pvL = new QHBoxLayout(pv);
	cmdLabel_ = new QLabel("---", pv);
	pvL->addWidget(cmdLabel_);
	root->addWidget(pv);

	// ---- Log ----
	QGroupBox * lf = new QGroupBox(tr("Log"), this);
	QVBoxLayout * lfL = new QVBoxLayout(lf);
	log_ = new QPlainTextEdit(lf);
	log_->setReadOnly(true);
	log_->setMaximumBlockCount(100);
	log_->setMaximumHeight(90);
	log_->setFocusPolicy(Qt::NoFocus);
	lfL->addWidget(log_);
	root->addWidget(lf);

	// ---- Keyboard help ----
	QLabel * help = new QLabel(
		tr("W/S speed (1s accel/decay) | A/D steer | Space all-stop"), this);
	help->setStyleSheet("color:gray;");
	root->addWidget(help);
	root->addStretch(1);

#ifndef RTABMAP_HAVE_QT_SERIALPORT
	statusLabel_->setText(tr("Qt SerialPort unavailable"));
	portCombo_->setEnabled(false);
	connectBtn_->setEnabled(false);
#endif
}

// ===========================================================================
// Serial I/O
// ===========================================================================
void ControllerPanel::refreshPorts()
{
	portCombo_->clear();
#ifdef RTABMAP_HAVE_QT_SERIALPORT
	const QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
	for(const QSerialPortInfo & info : ports)
		portCombo_->addItem(info.portName());
	if(portCombo_->count() > 0)
		portCombo_->setCurrentIndex(0);
#endif
}

void ControllerPanel::toggleConnect()
{
	if(connected_)
		disconnectSerial();
	else
		connectSerial();
}

void ControllerPanel::connectSerial()
{
#ifdef RTABMAP_HAVE_QT_SERIALPORT
	const QString port = portCombo_->currentText();
	if(port.isEmpty())
		return;
	if(!serial_)
		serial_ = new QSerialPort(this);
	serial_->setPortName(port);
	serial_->setBaudRate(BAUD_RATE);
	serial_->setDataBits(QSerialPort::Data8);
	serial_->setParity(QSerialPort::NoParity);
	serial_->setStopBits(QSerialPort::OneStop);
	serial_->setFlowControl(QSerialPort::NoFlowControl);
	if(serial_->open(QIODevice::ReadWrite))
	{
		connected_ = true;
		rxBuf_.clear();
		statusLabel_->setText(tr("connected"));
		statusLabel_->setStyleSheet("color:green;");
		connectBtn_->setText(tr("Disconnect"));
		connect(serial_, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
	}
	else
	{
		statusLabel_->setText(serial_->errorString());
		statusLabel_->setStyleSheet("color:red;");
	}
#endif
}

void ControllerPanel::disconnectSerial()
{
#ifdef RTABMAP_HAVE_QT_SERIALPORT
	if(serial_ && serial_->isOpen())
	{
		serial_->write("V0,A0\n");
		serial_->flush();
		serial_->close();
	}
#endif
	connected_ = false;
	if(statusLabel_)
	{
		statusLabel_->setText(tr("disconnected"));
		statusLabel_->setStyleSheet("color:gray;");
	}
	if(connectBtn_)
		connectBtn_->setText(tr("Connect"));
}

void ControllerPanel::onReadyRead()
{
#ifdef RTABMAP_HAVE_QT_SERIALPORT
	if(!serial_)
		return;
	rxBuf_ += serial_->readAll();
	int nl;
	while((nl = rxBuf_.indexOf('\n')) >= 0)
	{
		QString line = QString::fromLatin1(rxBuf_.left(nl)).trimmed();
		rxBuf_.remove(0, nl + 1);
		parseFeedback(line);
	}
#endif
}

void ControllerPanel::parseFeedback(const QString & line)
{
	if(line.isEmpty())
		return;
	if(!line.startsWith('E'))
	{
		appendLog(line);
		return;
	}
	const QStringList parts = line.split(',');
	for(const QString & part : parts)
	{
		if(part.isEmpty())
			continue;
		bool ok = false;
		const QChar tag = part.at(0);
		const int val = part.mid(1).toInt(&ok);
		if(!ok)
			continue;
		if(tag == 'E')      encDelta_ = val;
		else if(tag == 'P') potValue_ = val;
		else if(tag == 'C') encCumulative_ = val;
	}
}

void ControllerPanel::appendLog(const QString & msg)
{
	if(log_)
		log_->appendPlainText(msg);
}

// ===========================================================================
// Control loop
// ===========================================================================
void ControllerPanel::tick()
{
	rampSpeed();
	if(connected_)
		sendCommand();
	updateControlsDisplay();
	updateDisplay();
}

void ControllerPanel::sendCommand()
{
#ifdef RTABMAP_HAVE_QT_SERIALPORT
	if(!serial_ || !serial_->isOpen())
		return;
	const QByteArray cmd =
		QString("V%1,A%2\n").arg((int)std::lround(speed_)).arg(angle_).toLatin1();
	if(serial_->write(cmd) < 0)
		disconnectSerial();
#endif
}

void ControllerPanel::rampSpeed()
{
	const double target = targetSpeed_;
	if(speed_ == target)
		return;
	double rampMs;
	if(std::fabs(target) < std::fabs(speed_) || target * speed_ < 0)
		rampMs = SPEED_DECAY_MS;
	else
		rampMs = SPEED_ACCEL_MS;
	const double step = std::max(holdSpeed_, 1) * (double)SEND_INTERVAL_MS / rampMs;
	if(std::fabs(target - speed_) <= step)
		speed_ = target;
	else if(target > speed_)
		speed_ += step;
	else
		speed_ -= step;
}

// ===========================================================================
// GUI callbacks
// ===========================================================================
void ControllerPanel::onSpeedSlider(int v)
{
	if(updatingControls_)
		return;
	speed_ = v;
	targetSpeed_ = v;
	speedValLabel_->setText(QString::number(v));
}

void ControllerPanel::onAngleSlider(int v)
{
	if(updatingControls_)
		return;
	angle_ = v;
	angleValLabel_->setText(QString::number(v));
}

void ControllerPanel::resetSpeed()
{
	speed_ = 0.0;
	targetSpeed_ = 0.0;
	updatingControls_ = true;
	speedSlider_->setValue(0);
	updatingControls_ = false;
	speedValLabel_->setText("0");
}

void ControllerPanel::resetAngle()
{
	angle_ = 0;
	updatingControls_ = true;
	angleSlider_->setValue(0);
	updatingControls_ = false;
	angleValLabel_->setText("0");
}

void ControllerPanel::allStop()
{
	resetSpeed();
	resetAngle();
}

void ControllerPanel::syncHoldValues()
{
	holdSpeed_ = std::max(0, std::min(15, holdSpeedSpin_->value()));
	holdAngle_ = std::max(0, std::min(20, holdAngleSpin_->value()));
	applyHeldKeys();
}

void ControllerPanel::updateDisplay()
{
	encDeltaLabel_->setText(QString("Enc Delta: %1").arg(encDelta_));
	potLabel_->setText(QString("Pot (A0): %1").arg(potValue_));
	encCumLabel_->setText(QString("Enc Total: %1").arg(encCumulative_));
	cmdLabel_->setText(QString("V%1,A%2").arg((int)std::lround(speed_)).arg(angle_));
}

// ===========================================================================
// Keyboard control
// ===========================================================================
void ControllerPanel::keyPressEvent(QKeyEvent * event)
{
	if(event->isAutoRepeat())
	{
		event->accept();
		return;
	}
	if(event->key() == Qt::Key_Space)
	{
		allStop();
		event->accept();
		return;
	}
	keysPressed_.insert(event->key());
	applyHeldKeys();
	event->accept();
}

void ControllerPanel::keyReleaseEvent(QKeyEvent * event)
{
	if(event->isAutoRepeat())
	{
		event->accept();
		return;
	}
	keysPressed_.remove(event->key());
	applyHeldKeys();
	event->accept();
}

void ControllerPanel::applyHeldKeys()
{
	const bool fwd   = keysPressed_.contains(Qt::Key_W) || keysPressed_.contains(Qt::Key_Up);
	const bool back  = keysPressed_.contains(Qt::Key_S) || keysPressed_.contains(Qt::Key_Down);
	if(fwd == back)       targetSpeed_ = 0.0;
	else if(fwd)          targetSpeed_ = holdSpeed_;
	else                  targetSpeed_ = -holdSpeed_;

	const bool left  = keysPressed_.contains(Qt::Key_A) || keysPressed_.contains(Qt::Key_Left);
	const bool right = keysPressed_.contains(Qt::Key_D) || keysPressed_.contains(Qt::Key_Right);
	if(left == right)     angle_ = 0;
	else if(left)         angle_ = -holdAngle_;
	else                  angle_ = holdAngle_;

	updateControlsDisplay();
}

void ControllerPanel::updateControlsDisplay()
{
	const int spd = (int)std::lround(speed_);
	updatingControls_ = true;
	speedSlider_->setValue(spd);
	angleSlider_->setValue(angle_);
	updatingControls_ = false;
	speedValLabel_->setText(QString::number(spd));
	angleValLabel_->setText(QString::number(angle_));
}

} // namespace rtabmap
