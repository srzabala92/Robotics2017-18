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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <mutex>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define MAX_ADV 700
#define MAX_ROT 0.5
#define E 2.71828 
#define UMBRAL 250

enum class Estado{PARADO,AVANZANDO,GIRANDO,BORDEANDO, LLEGADO};
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	float gauss(float vr, float vx, float h );
        float sigmoide(float dis);

public slots:
	void compute(); 	

private:
      InnerModel* inner;
      std::pair<float, float> parxz , parIni ;//Valores (x,z) del objetivo e inicio
      QVec tR;//tR- Posicion del robot desde el pv del mundo 
      float d,A,B,C;//Distancia al objetivo/Para la recta: Ax+By+C=0
      float disRecta = 0.0;//Distancia del punto donde esta el robot a la recta
      Estado estado = Estado::PARADO;
      //Definimos una nueva estructura con 4 campos
      struct Target
      {
	QMutex mutex;
	bool empty =true;
	int x,y,z;
	bool isEmpty()
	{
	  QMutexLocker ml(&mutex);
	  return empty;
	};
	void setEmpty()
	{
	  QMutexLocker ml(&mutex);
	  empty = true;
	};
	void set(float x_, float z_)
	{
	  QMutexLocker ml(&mutex);
	  x = x_;
	  z = z_;
	  empty = false;
	}
	std::pair<float, float> get()
	{
	  QMutexLocker ml(&mutex);
	  return std::make_pair<float,float>(x,z);
	}	
      };
     
      
      Target target;
};

#endif

