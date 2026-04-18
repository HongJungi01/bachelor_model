/*
 * GridTcpStreamer.cpp — Implementation
 */

#include "rtabmap/gui/GridTcpStreamer.h"

#include <rtabmap/utilite/ULogger.h>

#include <QByteArray>

namespace rtabmap {

GridTcpStreamer::GridTcpStreamer(quint16 port, QObject * parent)
	: QObject(parent),
	  port_(port),
	  server_(new QTcpServer(this)),
	  minIntervalSec_(0.1)
{
	rateTimer_.start();
	connect(server_, SIGNAL(newConnection()), this, SLOT(onNewConnection()));
}

GridTcpStreamer::~GridTcpStreamer()
{
	stopListening();
}

bool GridTcpStreamer::isListening() const
{
	return server_->isListening();
}

int GridTcpStreamer::clientCount() const
{
	return clients_.size();
}

bool GridTcpStreamer::startListening()
{
	if(server_->isListening())
	{
		return true;
	}

	if(!server_->listen(QHostAddress::Any, port_))
	{
		UERROR("GridTcpStreamer: Failed to listen on port %d: %s",
		       (int)port_, server_->errorString().toStdString().c_str());
		Q_EMIT statusMessage(tr("TCP Grid: Failed to listen on port %1").arg(port_));
		return false;
	}

	UINFO("GridTcpStreamer: Listening on port %d", (int)port_);
	Q_EMIT statusMessage(tr("TCP Grid: Listening on port %1").arg(port_));
	return true;
}

void GridTcpStreamer::stopListening()
{
	// Disconnect all clients
	for(QTcpSocket * client : clients_)
	{
		client->disconnectFromHost();
		client->deleteLater();
	}
	clients_.clear();

	if(server_->isListening())
	{
		server_->close();
		UINFO("GridTcpStreamer: Stopped listening");
		Q_EMIT statusMessage(tr("TCP Grid: Stopped"));
	}
}

void GridTcpStreamer::onNewConnection()
{
	while(server_->hasPendingConnections())
	{
		QTcpSocket * socket = server_->nextPendingConnection();
		if(socket)
		{
			connect(socket, SIGNAL(disconnected()), this, SLOT(onClientDisconnected()));
			clients_.append(socket);
			UINFO("GridTcpStreamer: Client connected from %s:%d (total: %d)",
			      socket->peerAddress().toString().toStdString().c_str(),
			      (int)socket->peerPort(), clients_.size());
			Q_EMIT clientConnected(clients_.size());
			Q_EMIT statusMessage(tr("TCP Grid: %1 client(s) connected").arg(clients_.size()));
		}
	}
}

void GridTcpStreamer::onClientDisconnected()
{
	QTcpSocket * socket = qobject_cast<QTcpSocket*>(sender());
	if(socket)
	{
		clients_.removeAll(socket);
		socket->deleteLater();
		UINFO("GridTcpStreamer: Client disconnected (remaining: %d)", clients_.size());
		Q_EMIT clientDisconnected(clients_.size());
		Q_EMIT statusMessage(tr("TCP Grid: %1 client(s) connected").arg(clients_.size()));
	}
}

void GridTcpStreamer::sendGrid(const cv::Mat & map8S,
                               float xMin, float yMin, float cellSize,
                               float poseX, float poseY, float poseYaw)
{
	if(clients_.isEmpty() || map8S.empty())
	{
		return;
	}

	// Rate limiting
	double elapsed = rateTimer_.elapsed() / 1000.0;
	if(elapsed < minIntervalSec_)
	{
		return;
	}
	rateTimer_.restart();

	// Build packet
	GridPacketHeader hdr;
	std::memset(&hdr, 0, sizeof(hdr));
	hdr.width = map8S.cols;
	hdr.height = map8S.rows;
	hdr.xMin = xMin;
	hdr.yMin = yMin;
	hdr.cellSize = cellSize;
	hdr.poseX = poseX;
	hdr.poseY = poseY;
	hdr.poseYaw = poseYaw;

	size_t bodySize = static_cast<size_t>(map8S.cols) * map8S.rows;
	QByteArray packet;
	packet.resize(static_cast<int>(sizeof(hdr) + bodySize));
	std::memcpy(packet.data(), &hdr, sizeof(hdr));

	// Copy grid data row by row (handles non-contiguous mat)
	char * dst = packet.data() + sizeof(hdr);
	if(map8S.isContinuous())
	{
		std::memcpy(dst, map8S.data, bodySize);
	}
	else
	{
		for(int r = 0; r < map8S.rows; ++r)
		{
			std::memcpy(dst + r * map8S.cols, map8S.ptr<char>(r), map8S.cols);
		}
	}

	// Send to all clients, remove dead ones
	QList<QTcpSocket*> deadClients;
	for(QTcpSocket * client : clients_)
	{
		if(client->state() == QAbstractSocket::ConnectedState)
		{
			qint64 written = client->write(packet);
			if(written < 0)
			{
				deadClients.append(client);
			}
		}
		else
		{
			deadClients.append(client);
		}
	}
	for(QTcpSocket * dead : deadClients)
	{
		clients_.removeAll(dead);
		dead->disconnectFromHost();
		dead->deleteLater();
	}
}

} // namespace rtabmap
