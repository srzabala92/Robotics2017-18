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
#include "specificworker.h"
#include "time.h"
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{
    target.x = 0;
    target.y = 0;
    target.z = 0;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{
    inner = new InnerModel ( "/home/salabeta/robocomp/files/innermodel/simpleworld.xml" );
    timer.start ( Period );
    target.empty = true;

    return true;
}

void SpecificWorker::compute()
{
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState ( bState );
    RoboCompLaser::TLaserData datosLaser;
    datosLaser= laser_proxy->getLaserData();//Obtenemos datos del Laser
    inner->updateTransformValues ( "base",bState.x,0,bState.z,0,bState.alpha,0 );
   // datosLaser= laser_proxy->getLaserData();//Obtenemos datos del Laser
    std::sort(datosLaser.begin()+20, datosLaser.end()-20,[](auto a, auto b){return a.dist< b.dist;});//Ordenamos las distancias del frente
    tR = inner->transform ( "base" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
    d = tR.norm2(); // distancia del robot al punto marcado
    float vRot = atan2 ( tR.x(),tR.z() ); //devuelve radianes del angulo q forma donde apunta el robot con el punto destino.

    switch ( estado )
    {
    case Estado::PARADO:
        if ( target.isEmpty() == false )
        {
            //Calcular punto inicial,punto final,y trayectoria entre ellos
            // parIni = //Falta el punto inicial
            parxz = target.get();//Punto final
            tR = inner->transform ( "base" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
            d = tR.norm2(); // distancia del robot al punto marcado
            estado= Estado::AVANZANDO;
	    //Definir la recta - Solo se calcula al principio
	    //Coordenadas (x,y) del robot->Iniciales :bState.x,bState.z
	    //Coordenadas del Target ->Finales : parxz.first,parxz.second
	    A = parxz.second - bState.z ; 
	    B = parxz.first - bState.x;
	    C = -A * bState.x - B*bState.z;
	    
	    
	    
	    
        }
        break;
	
    case Estado::AVANZANDO:
      //Se recalcula la distancia segun avanza 
	 // qDebug() << "Distancia con la recta: " << disRecta ;

	//Cada vez que avance, se van obteniendo los datos actualizados del laser
// 	datosLaser= laser_proxy->getLaserData();//Obtenemos datos del Laser
// 	std::sort(datosLaser.begin()+20, datosLaser.end()-20,[](auto a, auto b){return a.dist< b.dist;});
	if (datosLaser[20].dist < UMBRAL){//Si hay obstaculo
	//Pasa a estado de Giro
	  estado = Estado::GIRANDO;
	  break ;
	  
	}
	
        if ( d > 50 )
        {
	  
            //Si no ha llegado
            float velAvance = d;// version antigua
//             float vRot = atan2 ( tR.x(),tR.z() ); //devuelve radianes del angulo q forma donde apunta el robot con el punto destino.

            if ( vRot > MAX_ROT )
            {
                vRot = MAX_ROT;
            }

            velAvance=MAX_ADV*sigmoide ( d ) *gauss ( vRot,0.3,0.5 );


            differentialrobot_proxy->setSpeedBase ( velAvance,vRot );
        }
        else
        {
            //Si ha llegado al sitio            
	    estado = Estado::LLEGADO;
        }
	
    
    break;
    
    
case Estado::GIRANDO:
    if (datosLaser[20].dist>UMBRAL)
    {
	estado=Estado::BORDEANDO;
    }
    else
    {
        //float d= rand()%1000000+100000;
	differentialrobot_proxy->setSpeedBase(0,0.75);
	//usleep(d); 
    }
      
   
	
    break;

case Estado::BORDEANDO:
   qDebug() << "Distancia con target: " << d;
   qDebug() << "Distancia con pared: " << datosLaser[20].dist;
 if (datosLaser[20].dist>UMBRAL && (vRot<0.03 && vRot > -0.03))//Que no haya obstaculos en el frente y vaya hacia el objetivo
    {
	estado=Estado::AVANZANDO;
    }
    if(datosLaser[20].dist<UMBRAL) //Si hay dos obstaculos en L, debe volver a girar para no tener obstaculo en frente
    {
      	estado=Estado::GIRANDO;
    }
  if ( d < 280)
  {
    estado=Estado::LLEGADO;
  }
//Condicion 2 para llegar al objetivo
 //if ( 
   //Ordenar los 20 primeros
   std::sort(datosLaser.end()-19, datosLaser.end()-10,[](auto a, auto b){return a.dist< b.dist;});//Valores de la izda
   
   //Evaluamos distancia izda 
  //qDebug() <<  "Distancia:" <<datosLaser[81].dist;
   if(datosLaser[81].dist < 400){//Que la distancia izda sea esa
     //Avanzar
     	differentialrobot_proxy->setSpeedBase(100,0);
   }
   else{
   differentialrobot_proxy->setSpeedBase(0,-0.75);
     
  }
  disRecta = abs(A*bState.x + B*bState.z + C)/ sqrt(pow(A,2) + pow(B,2));
  //qDebug() << "Distancia con la recta: " << disRecta ;
     break;
     
case Estado::LLEGADO:
    differentialrobot_proxy->setSpeedBase(0,0);
    target.setEmpty();
    estado=Estado::PARADO;
  
    break;

}

//    if ( target.isEmpty() == false )
//     {
//
//         std::pair<float, float> parxz = target.get();
//         QVec tR = inner->transform ( "robot" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
// 	float d = tR.norm2(); // distancia del robot al punto marcado
//
// 	if( d > 50 )//Si no ha llegado
// 	{
// 	  float velAvance = d;// version antigua
// 	  float vRot = atan2(tR.x(),tR.z()); //devuelve radianes del angulo q forma donde apunta el robot con el punto destino.
// 	/*  if (velAvance > MAX_ADV){
// 	    velAvance = MAX_ADV;
// 	  }*/
//
// 	  if( vRot > MAX_ROT){
// 	    vRot = MAX_ROT;
// 	  }
//
// 	  velAvance=MAX_ADV*sigmoide(d)*gauss(vRot,0.3,0.5);
//
//
// 	  differentialrobot_proxy->setSpeedBase(velAvance,vRot);
// 	  }
// 	else
// 	{ //Si ha llegado al sitio
// 	  differentialrobot_proxy->setSpeedBase(0,0);//Se ParameterList
// 	  target.setEmpty();
// 	}
//
//     }
// mtx.unlock();


}


void SpecificWorker::setPick ( const Pick &myPick )
{

    qDebug() <<  "x:" <<myPick.x;
    qDebug() <<  "y:" <<myPick.y;
    qDebug() <<  "z:" <<myPick.z;
    target.set ( myPick.x, myPick.z );
}

float SpecificWorker::gauss ( float vr, float vx, float h )
{
    float lambda=1.0;
    lambda= ( pow ( -vx,2.0 ) ) /log ( h );
    return pow ( E, ( pow ( -vr,2 ) ) /lambda );
}

float SpecificWorker::sigmoide ( float dis )
{
    return ( 1/ ( 1+pow ( E,-dis ) ) )-0.5;
}



