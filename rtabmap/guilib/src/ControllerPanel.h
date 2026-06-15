/*
 * ControllerPanel.h - "Default Controller" dock (quadrant 4).
 *
 * Native C++/Qt port of the project's Python default_Controller.py
 * (DebugController): a PID speed/steering debug panel that talks to the vehicle
 * Arduino over serial.
 *
 *   send:    V<speed>,A<angle>\n      (speed -15..15, angle -20..20)
 *   receive: E<delta>,P<pot>,C<cumulative>\n
 *
 * Serial uses Qt SerialPort when available (RTABMAP_HAVE_QT_SERIALPORT, set by
 * CMake when the qtserialport module is found); otherwise the panel still builds
 * and shows a "serial unavailable" status.
 *
 * Private guilib widget (header in src/, not exported); created in C++ by
 * MainWindow like the Semantic / Map+Path docks.
 */
#ifndef RTABMAP_GUI_CONTROLLERPANEL_H_
#define RTABMAP_GUI_CONTROLLERPANEL_H_

#include <QWidget>
#include <QByteArray>
#include <QSet>

class QComboBox;
class QPushButton;
class QLabel;
class QSlider;
class QSpinBox;
class QPlainTextEdit;
class QTimer;

#ifdef RTABMAP_HAVE_QT_SERIALPORT
class QSerialPort;
#endif

namespace rtabmap {

class ControllerPanel : public QWidget
{
	Q_OBJECT

public:
	explicit ControllerPanel(QWidget * parent = 0);
	virtual ~ControllerPanel();

protected:
	void keyPressEvent(QKeyEvent * event);
	void keyReleaseEvent(QKeyEvent * event);

private Q_SLOTS:
	void refreshPorts();
	void toggleConnect();
	void onReadyRead();
	void tick();
	void onSpeedSlider(int v);
	void onAngleSlider(int v);
	void resetSpeed();
	void resetAngle();
	void allStop();
	void syncHoldValues();

private:
	void buildGui();
	void connectSerial();
	void disconnectSerial();
	void parseFeedback(const QString & line);
	void appendLog(const QString & msg);
	void rampSpeed();
	void applyHeldKeys();
	void updateControlsDisplay();
	void updateDisplay();
	void sendCommand();

	// protocol / timing constants (== default_Controller.py)
	static const int BAUD_RATE       = 115200;
	static const int SEND_INTERVAL_MS = 20;
	static const int SPEED_ACCEL_MS  = 1000;
	static const int SPEED_DECAY_MS  = 1000;

	// widgets
	QComboBox *      portCombo_   = nullptr;
	QPushButton *    connectBtn_  = nullptr;
	QLabel *         statusLabel_ = nullptr;
	QSlider *        speedSlider_ = nullptr;
	QSlider *        angleSlider_ = nullptr;
	QLabel *         speedValLabel_ = nullptr;
	QLabel *         angleValLabel_ = nullptr;
	QSpinBox *       holdSpeedSpin_ = nullptr;
	QSpinBox *       holdAngleSpin_ = nullptr;
	QLabel *         encDeltaLabel_ = nullptr;
	QLabel *         potLabel_      = nullptr;
	QLabel *         encCumLabel_   = nullptr;
	QLabel *         cmdLabel_      = nullptr;
	QPlainTextEdit * log_          = nullptr;
	QTimer *         tickTimer_    = nullptr;

#ifdef RTABMAP_HAVE_QT_SERIALPORT
	QSerialPort *    serial_ = nullptr;
#endif
	QByteArray rxBuf_;

	// control state
	bool   connected_   = false;
	double speed_       = 0.0;   // -15..15 (float while ramping)
	int    angle_       = 0;     // -20..20
	double targetSpeed_ = 0.0;   // ramp target
	int    holdSpeed_   = 5;     // W/S target speed
	int    holdAngle_   = 15;    // A/D steering angle

	// feedback
	int encDelta_ = 0, potValue_ = 0, encCumulative_ = 0;

	QSet<int> keysPressed_;
	bool      updatingControls_ = false; // guards slider valueChanged loops
};

} // namespace rtabmap

#endif // RTABMAP_GUI_CONTROLLERPANEL_H_
