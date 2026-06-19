/*
 * AutopilotPanel.h - dashboard quadrant that replaces the in-GUI Controller.
 *
 * The controller now runs as a separate process (default_Controller.py): it
 * connects to rtabgui's PlanTcpStreamer, computes pure-pursuit and drives the
 * Arduino on its own steady loop. This panel is just the GUI side of that link —
 * it starts/stops the path+pose stream and shows its status (no serial, no manual
 * drive, no pursuit here).
 *
 * Private guilib widget (header in src/, not exported); created by MainWindow.
 */
#ifndef RTABMAP_GUI_AUTOPILOTPANEL_H_
#define RTABMAP_GUI_AUTOPILOTPANEL_H_

#include <QWidget>

class QCheckBox;
class QLabel;
class QTimer;

namespace rtabmap {

class PlanTcpStreamer;

class AutopilotPanel : public QWidget
{
	Q_OBJECT

public:
	explicit AutopilotPanel(PlanTcpStreamer * streamer, QWidget * parent = 0);

public Q_SLOTS:
	// MainWindow calls this when it streams, for a live data readout.
	void showStreamInfo(float px, float py, float yaw, int pathPts);

private Q_SLOTS:
	void onToggleStream(bool on);
	void onStatus(const QString & msg);
	void refresh();

private:
	void buildGui();

	PlanTcpStreamer * streamer_ = nullptr;
	QCheckBox * streamCheck_  = nullptr;
	QLabel *    statusLabel_  = nullptr;
	QLabel *    clientsLabel_ = nullptr;
	QLabel *    dataLabel_    = nullptr;
	QLabel *    msgLabel_     = nullptr;
	QTimer *    timer_        = nullptr;
};

} // namespace rtabmap

#endif // RTABMAP_GUI_AUTOPILOTPANEL_H_
