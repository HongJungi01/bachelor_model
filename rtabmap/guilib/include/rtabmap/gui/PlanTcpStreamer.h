/*
 * PlanTcpStreamer.h — TCP streaming of the planned path + live robot pose.
 *
 * Lets an external controller process (e.g. default_Controller.py) drive the
 * vehicle with its own steady real-time loop, immune to rtabgui lag. rtabgui only
 * publishes data here; pure-pursuit + serial live in the external controller.
 *
 * Wire format: newline-delimited ASCII text frames (trivial to parse in Python):
 *   pose:  "P <x> <y> <yaw>\n"                 (map frame, metres / radians)
 *   path:  "W <x1> <y1> <x2> <y2> ...\n"       (world waypoints, metres)
 *
 * Server side mirrors GridTcpStreamer (QTcpServer, broadcast to all clients).
 */
#ifndef RTABMAP_PLANTCPSTREAMER_H_
#define RTABMAP_PLANTCPSTREAMER_H_

#include "rtabmap/gui/rtabmap_gui_export.h"

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QList>
#include <QElapsedTimer>

#include <utility>
#include <vector>

namespace rtabmap {

class RTABMAP_GUI_EXPORT PlanTcpStreamer : public QObject
{
	Q_OBJECT

public:
	explicit PlanTcpStreamer(quint16 port = 5006, QObject * parent = nullptr);
	virtual ~PlanTcpStreamer();

	bool isListening() const;
	int clientCount() const;
	quint16 port() const { return port_; }

public Q_SLOTS:
	bool startListening();
	void stopListening();

	/// Broadcast the live map-frame pose (rate-limited).
	void sendPose(float x, float y, float yaw);
	/// Broadcast the latest planned path (world metres).
	void sendPath(const std::vector<std::pair<float, float> > & path);

Q_SIGNALS:
	void clientConnected(int totalClients);
	void clientDisconnected(int totalClients);
	void statusMessage(const QString & msg);

private Q_SLOTS:
	void onNewConnection();
	void onClientDisconnected();

private:
	void broadcast(const QByteArray & frame);

	quint16 port_;
	QTcpServer * server_;
	QList<QTcpSocket*> clients_;
	QElapsedTimer poseRateTimer_;
	double poseMinIntervalSec_;   // rate limit for pose frames (default 0.02 = 50Hz)
};

} // namespace rtabmap

#endif // RTABMAP_PLANTCPSTREAMER_H_
