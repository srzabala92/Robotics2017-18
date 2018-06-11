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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{
    tag.setId ( -1 );
    tag.set ( 0,0 );
    tag.setTiempo ( 0 );
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{

    timer.start ( Period );
    innermodel = new InnerModel ( "/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml" );

    return true;
}

void SpecificWorker::compute()
{
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState ( bState );
    innermodel->updateTransformValues ( "robot",bState.x,0,bState.z,0,bState.alpha,0 );
    qDebug() << "Estado: " << estado;
    //Solicitar a GetAPrilTags la lkista de tags
    obtenerTags();
    switch ( estado )
    {
    case 0://Estado 1 - Girando hasta encontrar un tag valido (una caja)
        if ( !noValidas.pertenece ( tag.getId() ) ) //Si la tag es valida (no es tag de pared o caja ya soltada)
        {
            qDebug() <<"Estoy yendo a la tag : "<< tag.getId();

            noValidas.insertarTag ( tag.getId() );
            irobjetivo_proxy->go ( tagInWorld.x(),tagInWorld.z() ); //Ir a Tag
            estado = 1;
        }
        else irobjetivo_proxy->turn ( 0.3 );
        break;
    case 1: //Encuentro primero caja
        if ( irobjetivo_proxy->getDistancia() < DIST_MIN)
        {
            qDebug() <<"He llegado";
            estado=2;

        }
        else irobjetivo_proxy->go ( tagInWorld.x(),tagInWorld.z() ); //Ir a Tag
        break;
    case 2:
        irobjetivo_proxy->stop();
	
	  irobjetivo_proxy->cogerCaja();
	  sleep ( 3 );
	  estado=3;
	  
	
	

        break;

    case 3:
        if ( tag.getId() ==3 )
        {
            irobjetivo_proxy->go ( tagInWorld.x(),tagInWorld.z() ); //Ir a Tag
            estado=4;
        }
        else
        {
            irobjetivo_proxy->turn ( 0.3 );
        }
        break;


    case 4: //Voy a la pared hasta...
        if ( irobjetivo_proxy->getDistancia() < ( DIST_MIN ) )
        {
            //irobjetivo_proxy->go ( 0,0 ); //quitar target
            estado=5;
        }
        break;

    case 5://Dejar caja en esquina
        irobjetivo_proxy->stop();
        irobjetivo_proxy->soltarCaja();
        sleep ( 4 );
        estado=6;
        break;
    case 6: //Comprobar si ha recogido todas las cajas
	if( noValidas.size() < 7){
	     estado = 0 ;
	}
	  break;
      
    }
 
}

bool SpecificWorker::obtenerTags(){
   QMutexLocker ml ( &mutexGlobal );
   try{
     tagsRecibidas.clear();
     for( auto t : getapriltags_proxy -> checkMarcas())
	  if ( t.id >= 3)
	  tagsRecibidas.push_back(t);
   }catch(const Ice::Exception &e) { std::printf("Error: Componente AprilTags no esta funcionando!\n");}
   if ( tagsRecibidas.empty()){
    qDebug() << "Lista Vacia";
     return false;
  }
  
  if ( tagsRecibidas.data()-> id > 2 )
    {
        qDebug() << "ME ha llegado la tag: " << tagsRecibidas.data()-> id;
        tag.set ( tagsRecibidas.data()->tx,tagsRecibidas.data()->tz );
        tag.setId ( tagsRecibidas.data()-> id );
        tagInWorld = innermodel->transform ( "world", QVec::vec3 ( tag.x,0,tag.z ),"robot" ); //Cambiamos a SRef del mundo*/
	//qDebug() << "Estoy viendo una caja";
    }
   return true;
}







