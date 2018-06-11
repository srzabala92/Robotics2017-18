/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "irobjetivoI.h"

IrObjetivoI::IrObjetivoI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


IrObjetivoI::~IrObjetivoI()
{
}

void IrObjetivoI::go(const float  x, const float  z, const Ice::Current&)
{
	worker->go(x, z);
}

void IrObjetivoI::turn(const float  speed, const Ice::Current&)
{
	worker->turn(speed);
}

float IrObjetivoI::getState(const Ice::Current&)
{
	return worker->getState();
}

void IrObjetivoI::stop(const Ice::Current&)
{
	worker->stop();
}






