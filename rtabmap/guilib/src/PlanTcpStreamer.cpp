/*
 * PlanTcpStreamer.cpp — see PlanTcpStreamer.h.
 *
 * Server/broadcast plumbing mirrors GridTcpStreamer; only the payload differs
 * (newline-delimited ASCII path/pose frames instead of the binary grid packet).
 */
#include "rtabmap/gui/PlanTcpStreamer.h"

#include <rtabmap/utilite/ULogger.h>

#include <QByteArray>
#include <QString>

namespace rtabmap {

PlanTcpStreamer::PlanTcpStreamer(quint16 port, QObject * parent)
	: QObject(parent),
	  port_(port),
	  server_(new QTcpServer(this)),
	  poseMinIntervalSec_(0.02)
{
	poseRateTimer_.start();
	connect(server_, SIGNAL(newConnection()), this, SLOT(onNewConnection()));
}

PlanTcpStreamer::~PlanTcpStreamer()
{
	stopListening();
}

bool PlanTcpStreamer::isListening() const
{
	return server_->isListening();
}

int PlanTcpStreamer::clientCount() const
{
	return clients_.size();
}

bool PlanTcpStreamer::startListening()
{
	if(server_->isListening())
		return true;

	if(!server_->listen(QHostAddress::Any, port_))
	{
		UERROR("PlanTcpStreamer: Failed to listen on port %d: %s",
		       (int)port_, server_->errorString().toStdString().c_str());
		Q_EMIT statusMessage(tr("Autopilot: failed to listen on port %1").arg(port_));
		return false;
	}
	UINFO("PlanTcpStreamer: Listening on port %d", (int)port_);
	Q_EMIT statusMessage(tr("Autopilot: listening on port %1").arg(port_));
	return true;
}

void PlanTcpStreamer::stopListening()
{
	for(QTcpSocket * client : clients_)
	{
		client->disconnectFromHost();
		client->deleteLater();
	}
	clients_.clear();

	if(server_->isListening())
	{
		server_->close();
		UINFO("PlanTcpStreamer: Stopped listening");
		Q_EMIT statusMessage(tr("Autopilot: stopped"));
	}
}

void PlanTcpStreamer::onNewConnection()
{
	while(server_->hasPendingConnections())
	{
		QTcpSocket * socket = server_->nextPendingConnection();
		if(socket)
		{
			connect(socket, SIGNAL(disconnected()), this, SLOT(onClientDisconnected()));
			clients_.append(socket);
			UINFO("PlanTcpStreamer: Client connected (total: %d)", clients_.size());
			Q_EMIT clientConnected(clients_.size());
			Q_EMIT statusMessage(tr("Autopilot: %1 controller(s) connected").arg(clients_.size()));
		}
	}
}

void PlanTcpStreamer::onClientDisconnected()
{
	QTcpSocket * socket = qobject_cast<QTcpSocket*>(sender());
	if(socket)
	{
		clients_.removeAll(socket);
		socket->deleteLater();
		UINFO("PlanTcpStreamer: Client disconnected (remaining: %d)", clients_.size());
		Q_EMIT clientDisconnected(clients_.size());
		Q_EMIT statusMessage(tr("Autopilot: %1 controller(s) connected").arg(clients_.size()));
	}
}

void PlanTcpStreamer::broadcast(const QByteArray & frame)
{
	if(clients_.isEmpty())
		return;
	QList<QTcpSocket*> dead;
	for(QTcpSocket * client : clients_)
	{
		if(client->state() == QAbstractSocket::ConnectedState)
		{
			if(client->write(frame) < 0)
				dead.append(client);
		}
		else
		{
			dead.append(client);
		}
	}
	for(QTcpSocket * d : dead)
	{
		clients_.removeAll(d);
		d->disconnectFromHost();
		d->deleteLater();
	}
}

void PlanTcpStreamer::sendPose(float x, float y, float yaw)
{
	if(clients_.isEmpty())
		return;
	if(poseRateTimer_.elapsed() / 1000.0 < poseMinIntervalSec_)
		return;
	poseRateTimer_.restart();
	broadcast(QString("P %1 %2 %3\n").arg(x, 0, 'f', 4).arg(y, 0, 'f', 4)
	          .arg(yaw, 0, 'f', 4).toLatin1());
}

void PlanTcpStreamer::sendPath(const std::vector<std::pair<float, float> > & path)
{
	if(clients_.isEmpty() || path.empty())
		return;
	QString s = "W";
	for(size_t i = 0; i < path.size(); ++i)
		s += QString(" %1 %2").arg(path[i].first, 0, 'f', 4).arg(path[i].second, 0, 'f', 4);
	s += "\n";
	broadcast(s.toLatin1());
}

} // namespace rtabmap
