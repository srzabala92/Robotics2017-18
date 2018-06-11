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
#include <list>

#define INCREMENT 10
#define MAX_ADV 700
#define MAX_ROT 0.5
#define ANGULO_VISION 0.03
#define E 2.71828 
#define UMBRAL 250
#define DIST_MIN 530

enum class Estado{PARADO,AVANZANDO,GIRANDO,BORDEANDO, LLEGADO};
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	float gauss(float vr, float vx, float h );
        float sigmoide(float dis);
	
	void stop();
	void soltarCaja();
	void turn(const float speed);
	void go(const float x, const float z);
	void cogerCaja();
	bool esVisible(int tag);
	float getDistancia();
	void setPick(const Pick &myPick);
	bool obtenerTags();
	void bajarBrazo();
	void bajarMano();
	void bajarMano(float grados);
	void subirBrazo(float grados);
	/**Jacobian**/
	void leftSlot();
	void rightSlot();
	void upSlot();
	void downSlot();
	void frontSlot();
	void backSlot();
	void goHome();
	void changeSpeed(int);
public slots:
	void compute(); 	

private:
      RoboCompLaser::TLaserData datosLaser;
      RoboCompDifferentialRobot::TBaseState bState;
      InnerModel* inner;
      std::pair<float, float> parxz , parIni ;//Valores (x,z) del objetivo e inicio
      QVec tR;//tR- Posicion del robot desde el pv del mundo 
      float d,vRot;//Distancia al objetivo
      Estado estado = Estado::PARADO;
      QVec tagInWorld;
      QMutex mutexGlobal;
      RoboCompJointMotor::MotorGoalPosition mg; 
   
  //Bajar muñeca
      
      //Estructura para almacenar etiquetas
      RoboCompGetAprilTags::listaMarcas tagsRecibidas;
      //Definimos una nueva estructura con 4 campos
      struct Target
      {
	QMutex mutex;
	bool empty =true , cambiado = true;//Con la variable "cambiado" controlamos que el target haya cambiado
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
	void setCambiado (bool cambiado_){
	  QMutexLocker ml(&mutex);
	  cambiado = cambiado_;
	}
	bool haCambiado(){
	  QMutexLocker ml(&mutex);
	  return cambiado;
	}
	std::pair<float, float> get()
	{
	  QMutexLocker ml(&mutex);
	  return std::make_pair<float,float>(x,z);
	}	
      };
        struct Tags{
      //Lista con las tags que llegan 
      std::vector<int> whiteList = { };
      QMutex mutex;
       bool isEmpty()
	{
	  QMutexLocker ml(&mutex);
	  return whiteList.empty();
	};
	void setEmpty()
	{
	  QMutexLocker ml(&mutex);
	  whiteList.clear();
	};
	/**Comprueba si una tag est´a o no **/
	bool pertenece(int tag){
	  QMutexLocker ml(&mutex);	  
	  for (auto t : whiteList){
	    if(t == tag)
	      return true;
	    
	  }
	  
	    return false;
	}
	/**Insertar tags en la lista**/
	/*void insertarTags(const tagsList &tags){
	  QMutexLocker ml(&mutex);
	  for (auto t:tags){
		  whiteList.push_back(t.id);
	  }
	}*/
	
     };
          struct Tag {
        QMutex mutex;
        float x,z;//Distancia robot - tag
        int id,tiempo;
        /*Introducir coordenadas distancia robot - tag
         */
        void set ( float x_, float z_ ) {
            QMutexLocker ml ( &mutex );
            x = x_;
            z = z_;
        }
        void setTiempo ( int tiempo_ ) {
            QMutexLocker ml ( &mutex );
            tiempo = tiempo_;
        }
        void setId ( int id_ ) {
            QMutexLocker ml ( &mutex );
            id = id_;
        }
        float getX() {
            QMutexLocker ml ( &mutex );
            return x;
        }
        float getZ() {
            QMutexLocker ml ( &mutex );
            return z;
        }
        int getId() {
            QMutexLocker ml ( &mutex );
            return id;
        }
        int getTiempo() {
            QMutexLocker ml ( &mutex );
            return tiempo;
        }
    };
      Target target;
      Tags tagsVistas;
      Tag tag;
      /**Jacobian**/
      RoboCompJointMotor::MotorParamsList mList;
      QStringList joints;
      QVec motores;
      QVec error;
      bool pushedButton = false;
      int FACTOR = 1;
      void parado();
      void avanzando();
      void bordeando();
      void girando();
      void llegado();
      void moveArm();
      void cerrarMano();
};

#endif

