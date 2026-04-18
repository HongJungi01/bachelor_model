/*
 * GridTcpStreamer.h — TCP streaming of 2D occupancy grid for Unity integration
 *
 * Streams the RTAB-Map occupancy grid over TCP using a 32-byte binary header
 * followed by raw grid data. Compatible with the existing rtabmap_2d_pipeline
 * protocol (GridPacketHeader + body).
 *
 * Protocol (little-endian):
 *   Header (32 bytes):
 *     int32  width, height
 *     float  xMin, yMin, cellSize
 *     float  poseX, poseY, poseYaw
 *   Body (width * height bytes):
 *     int8 per cell: -1=unknown, 0=free, 100=obstacle
 */

#ifndef RTABMAP_GRIDTCPSTREAMER_H_
#define RTABMAP_GRIDTCPSTREAMER_H_

#include "rtabmap/gui/rtabmap_gui_export.h"

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QList>
#include <QElapsedTimer>

#include <opencv2/core.hpp>

#include <cstdint>
#include <cstring>

namespace rtabmap {

#pragma pack(push, 1)
struct GridPacketHeader {
	int32_t  width;      // grid columns
	int32_t  height;     // grid rows
	float    xMin;       // world X of grid origin (metres)
	float    yMin;       // world Y of grid origin (metres)
	float    cellSize;   // metres per cell
	float    poseX;      // current robot X (metres)
	float    poseY;      // current robot Y (metres)
	float    poseYaw;    // current robot heading (radians)
};
#pragma pack(pop)
static_assert(sizeof(GridPacketHeader) == 32, "GridPacketHeader must be 32 bytes");

class RTABMAP_GUI_EXPORT GridTcpStreamer : public QObject
{
	Q_OBJECT

public:
	explicit GridTcpStreamer(quint16 port = 7777, QObject * parent = nullptr);
	virtual ~GridTcpStreamer();

	bool isListening() const;
	int clientCount() const;
	quint16 port() const { return port_; }
	void setMinInterval(double seconds) { minIntervalSec_ = seconds; }

public Q_SLOTS:
	/// Start listening on the configured port. Returns true on success.
	bool startListening();
	/// Stop listening and disconnect all clients.
	void stopListening();

	/**
	 * Send grid data to all connected clients.
	 * @param map8S  CV_8SC1 occupancy grid (-1=unknown, 0=free, 100=obstacle)
	 * @param xMin   world X origin of the grid (metres)
	 * @param yMin   world Y origin of the grid (metres)
	 * @param cellSize cell size in metres
	 * @param poseX  current robot X (metres)
	 * @param poseY  current robot Y (metres)
	 * @param poseYaw current robot heading (radians)
	 */
	void sendGrid(const cv::Mat & map8S,
	              float xMin, float yMin, float cellSize,
	              float poseX, float poseY, float poseYaw);

Q_SIGNALS:
	void clientConnected(int totalClients);
	void clientDisconnected(int totalClients);
	void statusMessage(const QString & msg);

private Q_SLOTS:
	void onNewConnection();
	void onClientDisconnected();

private:
	quint16 port_;
	QTcpServer * server_;
	QList<QTcpSocket*> clients_;
	QElapsedTimer rateTimer_;
	double minIntervalSec_;   // rate limiting (default 0.1 = 10Hz)
};

} // namespace rtabmap

#endif // RTABMAP_GRIDTCPSTREAMER_H_
