/*
 * AutopilotPanel.cpp - see AutopilotPanel.h.
 */
#include "AutopilotPanel.h"
#include "rtabmap/gui/PlanTcpStreamer.h"

#include <QCheckBox>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>

namespace rtabmap {

AutopilotPanel::AutopilotPanel(PlanTcpStreamer * streamer, QWidget * parent) :
	QWidget(parent),
	streamer_(streamer)
{
	buildGui();

	if(streamer_)
	{
		connect(streamer_, SIGNAL(clientConnected(int)),    this, SLOT(refresh()));
		connect(streamer_, SIGNAL(clientDisconnected(int)), this, SLOT(refresh()));
		connect(streamer_, SIGNAL(statusMessage(QString)),  this, SLOT(onStatus(QString)));
		streamCheck_->setChecked(streamer_->isListening());
	}

	timer_ = new QTimer(this);
	connect(timer_, SIGNAL(timeout()), this, SLOT(refresh()));
	timer_->start(500);
	refresh();
}

void AutopilotPanel::buildGui()
{
	QVBoxLayout * root = new QVBoxLayout(this);
	root->setContentsMargins(8, 8, 8, 8);
	root->setSpacing(6);

	QGroupBox * box = new QGroupBox(tr("Autopilot — path+pose stream"), this);
	QVBoxLayout * v = new QVBoxLayout(box);

	streamCheck_ = new QCheckBox(tr("Stream path + pose to controller"), box);
	streamCheck_->setFocusPolicy(Qt::NoFocus);
	v->addWidget(streamCheck_);
	connect(streamCheck_, SIGNAL(toggled(bool)), this, SLOT(onToggleStream(bool)));

	statusLabel_  = new QLabel(tr("status: --"), box);
	clientsLabel_ = new QLabel(tr("controllers connected: 0"), box);
	dataLabel_    = new QLabel(tr("last: pose --   path -- pts"), box);
	msgLabel_     = new QLabel("", box);
	msgLabel_->setStyleSheet("color:gray;");
	msgLabel_->setWordWrap(true);
	v->addWidget(statusLabel_);
	v->addWidget(clientsLabel_);
	v->addWidget(dataLabel_);
	v->addWidget(msgLabel_);
	root->addWidget(box);

	QLabel * help = new QLabel(
		tr("Controller runs as a separate process (default_Controller.py): it "
		   "connects here, computes pure-pursuit and drives the vehicle on its own "
		   "loop — unaffected by GUI lag."), this);
	help->setStyleSheet("color:gray;");
	help->setWordWrap(true);
	root->addWidget(help);
	root->addStretch(1);
}

void AutopilotPanel::onToggleStream(bool on)
{
	if(!streamer_)
		return;
	if(on)
		streamer_->startListening();
	else
		streamer_->stopListening();
	refresh();
}

void AutopilotPanel::onStatus(const QString & msg)
{
	if(msgLabel_)
		msgLabel_->setText(msg);
}

void AutopilotPanel::refresh()
{
	if(!streamer_)
		return;
	const bool listening = streamer_->isListening();
	statusLabel_->setText(listening
		? tr("status: listening on port %1").arg(streamer_->port())
		: tr("status: stopped"));
	statusLabel_->setStyleSheet(listening ? "color:green;" : "color:gray;");
	clientsLabel_->setText(tr("controllers connected: %1").arg(streamer_->clientCount()));
}

void AutopilotPanel::showStreamInfo(float px, float py, float yaw, int pathPts)
{
	if(dataLabel_)
		dataLabel_->setText(tr("last: pose (%1, %2) yaw %3   path %4 pts")
			.arg(px, 0, 'f', 2).arg(py, 0, 'f', 2).arg(yaw, 0, 'f', 2).arg(pathPts));
}

} // namespace rtabmap
