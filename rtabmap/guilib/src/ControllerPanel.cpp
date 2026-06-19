/*
 * ControllerPanel.cpp - see ControllerPanel.h.
 *
 * 1:1 behavioural port of default_Controller.py: 20 ms control tick with speed
 * ramping (1 s accel / 1 s decay), W/S/A/D + Space keyboard control, sliders,
 * hold-value spinboxes, E-stop, live E/P/C feedback and a command log.
 */
#include "ControllerPanel.h"
#include "MapPathView.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
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
#include <utility>
#include <vector>

#ifdef RTABMAP_HAVE_QT_SERIALPORT
#include <QSerialPort>
#include <QSerialPortInfo>
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX            // keep windows.h from clobbering std::max/std::min used below
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
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

	// ---- Auto mode (path following) ----
	QGroupBox * autof = new QGroupBox(tr("Auto mode (Map+Path following)"), this);
	QHBoxLayout * autoL = new QHBoxLayout(autof);
	autoModeCheck_ = new QCheckBox(tr("Auto"), autof);
	autoModeCheck_->setFocusPolicy(Qt::NoFocus);
	autoL->addWidget(autoModeCheck_);
	autoL->addSpacing(12);
	autoL->addWidget(new QLabel(tr("Auto speed:"), autof));
	autoSpeedSpin_ = new QSpinBox(autof);
	autoSpeedSpin_->setRange(0, 15);
	autoSpeedSpin_->setValue(autoSpeed_);
	autoSpeedSpin_->setFocusPolicy(Qt::ClickFocus);
	autoL->addWidget(autoSpeedSpin_);
	autoL->addSpacing(12);
	autoL->addWidget(new QLabel(tr("LAD (m):"), autof));
	lookAheadSpin_ = new QDoubleSpinBox(autof);
	lookAheadSpin_->setRange(0.1, 5.0);
	lookAheadSpin_->setSingleStep(0.1);
	lookAheadSpin_->setDecimals(2);
	lookAheadSpin_->setValue(lookAhead_);
	lookAheadSpin_->setFocusPolicy(Qt::ClickFocus);
	autoL->addWidget(lookAheadSpin_);
	autoL->addStretch(1);
	root->addWidget(autof);
	connect(autoModeCheck_, SIGNAL(toggled(bool)), this, SLOT(onAutoModeToggled(bool)));
	connect(autoSpeedSpin_, SIGNAL(valueChanged(int)), this, SLOT(syncAutoValues()));
	connect(lookAheadSpin_, SIGNAL(valueChanged(double)), this, SLOT(syncAutoValues()));

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

#if !defined(RTABMAP_HAVE_QT_SERIALPORT) && !defined(_WIN32)
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
#elif defined(_WIN32)
	// Enumerate COM ports via the DOS device map (kernel32 only, no advapi32 link).
	QByteArray buf(65536, '\0');
	const DWORD len = QueryDosDeviceA(nullptr, buf.data(), (DWORD)buf.size());
	if(len > 0)
	{
		// QueryDosDevice returns a double-null-terminated list of NUL-separated names.
		const char * p = buf.constData();
		const char * end = p + len;
		while(p < end && *p)
		{
			const QString name = QString::fromLatin1(p);
			p += qstrlen(p) + 1;
			if(!name.startsWith("COM"))
				continue;
			bool digits = name.size() > 3;
			for(int i = 3; digits && i < name.size(); ++i)
				digits = name.at(i).isDigit();
			if(digits)
				portCombo_->addItem(name);
		}
	}
	portCombo_->model()->sort(0);
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
#elif defined(_WIN32)
	const QString port = portCombo_->currentText();
	if(port.isEmpty())
		return;
	// "\\.\COMx" device path (the prefix is required for COM10 and above).
	const QString path = "\\\\.\\" + port;
	HANDLE h = CreateFileA(path.toLatin1().constData(),
	                       GENERIC_READ | GENERIC_WRITE, 0, nullptr,
	                       OPEN_EXISTING, 0, nullptr);
	if(h == INVALID_HANDLE_VALUE)
	{
		statusLabel_->setText(tr("open failed (err %1)").arg((qulonglong)GetLastError()));
		statusLabel_->setStyleSheet("color:red;");
		return;
	}
	DCB dcb;
	ZeroMemory(&dcb, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	GetCommState(h, &dcb); // seed from current state; dcb stays zero-initialized on failure
	dcb.BaudRate = BAUD_RATE;
	dcb.ByteSize = 8;
	dcb.Parity   = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fBinary  = TRUE;
	dcb.fParity  = FALSE;
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;
	dcb.fDtrControl  = DTR_CONTROL_ENABLE;
	dcb.fRtsControl  = RTS_CONTROL_ENABLE;
	if(!SetCommState(h, &dcb))
	{
		statusLabel_->setText(tr("config failed (err %1)").arg((qulonglong)GetLastError()));
		statusLabel_->setStyleSheet("color:red;");
		CloseHandle(h);
		return;
	}
	// Non-blocking reads: return immediately with whatever bytes are buffered.
	COMMTIMEOUTS to;
	ZeroMemory(&to, sizeof(to));
	to.ReadIntervalTimeout = MAXDWORD;
	to.ReadTotalTimeoutConstant = 0;
	to.ReadTotalTimeoutMultiplier = 0;
	to.WriteTotalTimeoutConstant = 50;
	to.WriteTotalTimeoutMultiplier = 0;
	SetCommTimeouts(h, &to);
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	winSerial_ = h;
	connected_ = true;
	rxBuf_.clear();
	statusLabel_->setText(tr("connected"));
	statusLabel_->setStyleSheet("color:green;");
	connectBtn_->setText(tr("Disconnect"));
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
#elif defined(_WIN32)
	if(winSerial_)
	{
		HANDLE h = (HANDLE)winSerial_;
		DWORD written = 0;
		WriteFile(h, "V0,A0\n", 6, &written, nullptr);
		FlushFileBuffers(h);
		CloseHandle(h);
		winSerial_ = nullptr;
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
	drainRxBuffer();
#endif
}

// Splits complete '\n'-terminated lines out of rxBuf_ and parses each. The
// caller is responsible for filling rxBuf_ (Qt readyRead or Win32 tick poll).
void ControllerPanel::drainRxBuffer()
{
	int nl;
	while((nl = rxBuf_.indexOf('\n')) >= 0)
	{
		QString line = QString::fromLatin1(rxBuf_.left(nl)).trimmed();
		rxBuf_.remove(0, nl + 1);
		parseFeedback(line);
	}
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
	if(autoMode_)
		applyAutoControl();
	rampSpeed();
	if(connected_)
		sendCommand();
#if defined(_WIN32) && !defined(RTABMAP_HAVE_QT_SERIALPORT)
	// No readyRead signal on the Win32 path: drain the OS receive buffer here.
	if(connected_ && winSerial_)
	{
		char tmp[512];
		DWORD got = 0;
		while(ReadFile((HANDLE)winSerial_, tmp, sizeof(tmp), &got, nullptr) && got > 0)
		{
			rxBuf_ += QByteArray(tmp, (int)got);
			if(got < sizeof(tmp))
				break;
		}
		drainRxBuffer();
	}
#endif
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
#elif defined(_WIN32)
	if(!winSerial_)
		return;
	const QByteArray cmd =
		QString("V%1,A%2\n").arg((int)std::lround(speed_)).arg(angle_).toLatin1();
	DWORD written = 0;
	if(!WriteFile((HANDLE)winSerial_, cmd.constData(), (DWORD)cmd.size(), &written, nullptr))
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
	// Big-red-button / Space also exits auto mode (emergency manual takeover).
	if(autoModeCheck_ && autoModeCheck_->isChecked())
		autoModeCheck_->setChecked(false); // -> onAutoModeToggled(false)
	resetSpeed();
	resetAngle();
}

void ControllerPanel::syncHoldValues()
{
	holdSpeed_ = std::max(0, std::min(15, holdSpeedSpin_->value()));
	holdAngle_ = std::max(0, std::min(20, holdAngleSpin_->value()));
	applyHeldKeys();
}

void ControllerPanel::syncAutoValues()
{
	if(autoSpeedSpin_)
		autoSpeed_ = std::max(0, std::min(15, autoSpeedSpin_->value()));
	if(lookAheadSpin_)
		lookAhead_ = lookAheadSpin_->value();
}

void ControllerPanel::onAutoModeToggled(bool on)
{
	autoMode_ = on;
	// Lock out manual driving widgets while the path is in control.
	if(speedSlider_)   speedSlider_->setEnabled(!on);
	if(angleSlider_)   angleSlider_->setEnabled(!on);
	if(holdSpeedSpin_) holdSpeedSpin_->setEnabled(!on);
	if(holdAngleSpin_) holdAngleSpin_->setEnabled(!on);
	keysPressed_.clear();
	if(on)
	{
		syncAutoValues();
		appendLog(tr("Auto mode ON (path following)"));
	}
	else
	{
		targetSpeed_ = 0.0; // hand back to manual, stopped
		angle_ = 0;
		updateControlsDisplay();
		appendLog(tr("Auto mode OFF"));
	}
	setFocus();
}

// Auto mode: each tick, steer along the Map+Path plan and hold the auto speed.
void ControllerPanel::applyAutoControl()
{
	int steer = 0;
	if(computeAutoSteering(steer))
	{
		angle_ = steer;
		targetSpeed_ = autoSpeed_;
	}
	else
	{
		angle_ = 0;          // no valid plan/pose yet -> straight & stopped (safety)
		targetSpeed_ = 0.0;
	}
	updateControlsDisplay();
}

// Pure-pursuit steering from MapPathView's world path + live pose, mapped to -20..20.
bool ControllerPanel::computeAutoSteering(int & angleOut)
{
	if(!mapPath_)
		return false;
	std::vector<std::pair<float, float> > path;
	float px = 0.f, py = 0.f, yaw = 0.f;
	if(!mapPath_->latestPlanAndPose(path, px, py, yaw) || path.size() < 2)
		return false;

	const double lad = std::max(0.05, lookAhead_);
	// 1) nearest path point (current progress along the plan).
	int nearest = 0;
	double bestD = -1.0;
	for(size_t i = 0; i < path.size(); ++i)
	{
		const double dx = path[i].first - px, dy = path[i].second - py;
		const double d = dx*dx + dy*dy;
		if(bestD < 0.0 || d < bestD) { bestD = d; nearest = (int)i; }
	}
	// 2) first point >= LAD ahead of that. Scanning from index 0 would latch onto a
	//    point behind the robot once it advanced (path is ~2 s stale) -> |alpha|~180°
	//    -> saturated wrong-way steering.
	int idx = -1;
	for(int i = nearest; i < (int)path.size(); ++i)
	{
		const double dx = path[i].first - px, dy = path[i].second - py;
		if(std::sqrt(dx*dx + dy*dy) >= lad) { idx = i; break; }
	}
	if(idx < 0)
		idx = (int)path.size() - 1; // remaining path within LAD -> aim at the goal

	const double kPi = 3.14159265358979323846;
	double alpha = std::atan2((double)path[idx].second - py,
	                          (double)path[idx].first  - px) - (double)yaw;
	while(alpha >  kPi) alpha -= 2.0 * kPi;
	while(alpha < -kPi) alpha += 2.0 * kPi;

	// Heading error -> steering units: |alpha| >= FULL_LOCK_DEG saturates at the limit.
	// Left (alpha>0, CCW) maps negative to match manual A=left. Tune the feel via LAD.
	const double FULL_LOCK_DEG = 30.0;
	double cmd = -(alpha * 180.0 / kPi) / FULL_LOCK_DEG * 20.0;
	int a = (int)std::lround(cmd);
	if(a >  20)      a =  20;
	else if(a < -20) a = -20;
	angleOut = a;
	return true;
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
	if(autoMode_) // manual W/S/A/D ignored while the path drives
		return;
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
