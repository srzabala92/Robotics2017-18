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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    inner = new InnerModel ( "/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml" );
    timer.start ( Period );
    target.empty = true; 
    //subirBrazo(0);
    /**Jacobian**/
    goHome();
    sleep(1);
    try { 
      mList = jointmotor_proxy->getAllMotorParams();}
    catch(const Ice::Exception &e){ std::cout << e << std::endl;}
    joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right" << "wrist_right_1" << "wrist_right_2";
    // Check that these names are in mList
    motores = QVec::zeros(joints.size());
    timer.start(100);
    subirBrazo(0.40);
    return true;
}

void SpecificWorker::compute()
{

    differentialrobot_proxy->getBaseState ( bState );
    
    datosLaser= laser_proxy->getLaserData();//Obtenemos datos del Laser
    inner->updateTransformValues ( "robot",bState.x,0,bState.z,0,bState.alpha,0 );
    // datosLaser= laser_proxy->getLaserData();//Obtenemos datos del Laser
    std::sort ( datosLaser.begin() +20, datosLaser.end()-20,[] ( auto a, auto b )
    {
        return a.dist< b.dist;
    } ); //Ordenamos las distancias del frente
    tR = inner->transform ( "robot" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
    d = tR.norm2(); // distancia del robot al punto marcado
    vRot = atan2 ( tR.x(),tR.z() ); //devuelve radianes del angulo q forma donde apunta el robot con el punto destino.
    //Refresca el estado
     if ( target.isEmpty() == false && target.haCambiado())
        {
            //Calcular punto inicial,punto final,y trayectoria entre ellos
            // parIni = //Falta el punto inicial
            parxz = target.get();//Punto final
            tR = inner->transform ( "robot" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
            d = tR.norm2(); // distancia del robot al punto marcado
            estado= Estado::AVANZANDO;
            target.setCambiado(false);

        }

    
    switch ( estado )
    {
    case Estado::PARADO:
        qDebug() << "PARADO";
	parado();
        break;

    case Estado::AVANZANDO:
         qDebug() << "AVANZANDO";
	avanzando();

        break;


    case Estado::GIRANDO:
	 qDebug() << "GIRANDO";
	 girando();

        break;

    case Estado::BORDEANDO:
        qDebug() << "BORDEANDO";
	bordeando();
	break;

    case Estado::LLEGADO:
         qDebug() << "LLEGADO";
	llegado();
        break;

    }
    
}




void SpecificWorker::soltarCaja()
{
  bajarBrazo();
}


/*
 *Implementacion del metodo 'go' de la interfaz "IrObjetivo"
 * Si el target estaba vacio,lo define con las coordenadas x,z que se pasan 
 * por parametro.
 * Si no:
 * 	-Si ha variado la x o la z en mas de 50 (o -50)
 * 	se refresca el valor del target
 * 	-Si no: El target se mantiene (no hace nada)
 * 
 */ 
void SpecificWorker::go ( const float x, const float z )
{
  if ( target.isEmpty())
  {
    target.set ( x, z );
    target.setCambiado(true);
  }
  else {
  int dX = 0 , dZ = 0; //Diferencias de X y Z
  dX = x - target.x;
  dZ = z - target.z;
  if ( abs(dX) > 50 || abs(dZ) > 50 ){
    qDebug()<<"Target Actualizado!---------------------------------------------------------------------";
    target.set ( x, z );
    target.setCambiado(true);
    }
  }
}


/*void SpecificWorker::cogerCaja()
{
	bajarMano();
	RoboCompJointMotor::MotorStateMap mMap;
	float difTX  = 0 ,difTY = 0, difTZ = 0;
	obtenerTags();
	
 	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
			inner->updateJointValue(QString::fromStdString(m.first),m.second.pos);
			//std::cout << m.first << "		" << m.second.pos << std::endl;
		}
		std::cout << "--------------------------" << std::endl;
	}
	catch(const Ice::Exception &e)
	{	std::cout << "JoinMotor -- "<<e.what() << std::endl;}
	
	//Compute Jacobian for the chain of joints and using as tip "cameraHand" 

	QMat jacobian = inner->jacobian(joints, motores, "rgbdHand");
	qDebug() << "3";
	RoboCompJointMotor::MotorGoalVelocityList vl;
	error = QVec::vec6(0,0,0,0,0,0);
	if (!tagsRecibidas.empty()){
		qDebug() << "ENTRANDO A COGER CAJA--------------";
		try
		{
		  
		  
			  difTX = tagsRecibidas.front().tx; //objetivo final 
			  difTY = tagsRecibidas.front().ty;
			  difTZ = tagsRecibidas.front().tz;
			  qDebug() << "difTX: "<< difTX << " difTY: " << difTY << "DifTZ: " << difTZ;
			  // ESTO ES CORRECTO
			  if (difTX > INCREMENT){
			    rightSlot();
			  }
			  if (difTX < -INCREMENT){
			    leftSlot();
			  }
			  if (difTY > INCREMENT){
			    frontSlot();
			  }
			  if (difTY < -INCREMENT){
			    backSlot();
			  }
			  if (difTZ > INCREMENT){
			    downSlot();
			  }
			  if (difTZ < -INCREMENT){
			    upSlot();
			  }
			  
			
			QVec incs = jacobian.invert() * error;	
			int i=0;
			for(auto m: joints)
			{
				//RoboCompJointMotor::MotorGoalPosition mg = {mMap.find(m.toStdString())->second.pos + incs[i], 1.0, m.toStdString()};
				RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
				//ml.push_back(mg);
				vl.push_back(vg);
				i++;
			}	//Aplicar cambios al brazo
				try
				  { 
					  qDebug()<<"Moviendo Brazo...";
					  jointmotor_proxy->setSyncVelocity(vl);
				  }
				  catch(const Ice::Exception &e)
				  {	std::cout << "SetSyncVelocity "<<e.what() << std::endl;}
				  qDebug() << "Cerrando la mano...";
				  cerrarMano();
				 
		  
		}catch(const QString &e)
		{ qDebug() << e << "Error inverting matrix";}
	  }
	  
}*/
void SpecificWorker::cogerCaja()
{
bajarMano();
cerrarMano();
}


bool SpecificWorker::esVisible(int tag_caja)
{
  bajarMano();
  
  if(obtenerTags())
  {
  qDebug()<<"Tag que esta viendo: "<<tag.getId();
  if (tag.getId() == tag_caja)
    return true;
  }

  return false;
}

float SpecificWorker::getDistancia()
{
  return d;
}

void SpecificWorker::setPick(const Pick &myPick)
{
    qDebug() <<  "x:" <<myPick.x;
    qDebug() <<  "y:" <<myPick.y;
    qDebug() <<  "z:" <<myPick.z;
    target.setCambiado(true);
    target.set ( myPick.x, myPick.z );
}
//Subscripicion a AprilTagsmano


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

void SpecificWorker::turn ( const float speed )
{
  differentialrobot_proxy->setSpeedBase ( 0,speed );
}


void SpecificWorker::stop()
{
   differentialrobot_proxy->setSpeedBase ( 0,0 );
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
        tagInWorld = inner->transform ( "world", QVec::vec3 ( tag.x,0,tag.z ),"robot" ); //Cambiamos a SRef del mundo*/
	//qDebug() << "Estoy viendo una caja";
    }
   return true;
}
void SpecificWorker::bajarBrazo(){
   differentialrobot_proxy->setSpeedBase(0 , 0);

    RoboCompJointMotor::MotorGoalPosition shoulder_right_2;
    shoulder_right_2.name = "shoulder_right_2";
    shoulder_right_2.position = -0.35;
    shoulder_right_2.maxSpeed = 0.5;
    jointmotor_proxy->setPosition(shoulder_right_2);
    sleep(3);
  
    goHome();
   
  
}
void SpecificWorker::bajarMano(){
    mg.name="wrist_right_2";
    mg.position=1.30;
    mg.maxSpeed=0.3;
    jointmotor_proxy->setPosition(mg);
    sleep(1);
}
void SpecificWorker::bajarMano(float grados){
    mg.name="wrist_right_2";
    mg.position=mg.position - grados;
    mg.maxSpeed=0;
    jointmotor_proxy->setPosition(mg);
   // sleep(1);
}
void SpecificWorker::subirBrazo(float grados)
{
    mg.name="shoulder_right_2";
    mg.position=-mg.position - grados;
    mg.maxSpeed=0;
    jointmotor_proxy->setPosition(mg);
}

 void SpecificWorker::parado(){
        if ( target.isEmpty() == false)
        {
            //Calcular punto inicial,punto final,y trayectoria entre ellos
            // parIni = //Falta el punto inicial
            parxz = target.get();//Punto final
            tR = inner->transform ( "robot" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
            d = tR.norm2(); // distancia del robot al punto marcado
            estado= Estado::AVANZANDO;
            target.setCambiado(false);

        }
 }
 void SpecificWorker::avanzando(){
   
        if ( datosLaser[20].dist < UMBRAL ) //Si hay obstaculo
        {
            //Pasa a estado de Giro
            estado = Estado::GIRANDO;
            return ;

        }

        if ( d > DIST_MIN )
        {

            //Si no ha llegado
            float velAvance = d;// version antigua
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

}
 void SpecificWorker::bordeando(){
      if ( datosLaser[20].dist>UMBRAL && ( vRot<ANGULO_VISION && vRot > -(ANGULO_VISION) )) //Que no haya obstaculos en el frente y vaya hacia el objetivo
        {
            estado=Estado::AVANZANDO;
        }
        if ( datosLaser[20].dist<UMBRAL ) //Si hay dos obstaculos en L, debe volver a girar para no tener obstaculo en frente
        {
            estado=Estado::GIRANDO;
        }
        if ( d < 580 )
        {
            estado=Estado::LLEGADO;
        }
        //Condicion 2 para llegar al objetivo
        //if (
        //Ordenar los 20 primeros
        std::sort ( datosLaser.end()-19, datosLaser.end()-10,[] ( auto a, auto b )
        {
            return a.dist< b.dist;
        } ); //Valores de la izda

        //Evaluamos distancia izda
        //qDebug() <<  "Distancia:" <<datosLaser[81].dist;
        if ( datosLaser[81].dist < 400 ) //Que la distancia izda sea esa
        {
            //Avanzar
            differentialrobot_proxy->setSpeedBase ( 100,0 );
        }
        else
        {
            differentialrobot_proxy->setSpeedBase ( 0,-0.75 );

        }
}
 void SpecificWorker::girando(){
   
        if ( datosLaser[20].dist>UMBRAL )
        {
            estado=Estado::BORDEANDO;
        }
        else
        {
            //float d= rand()%1000000+100000;
            differentialrobot_proxy->setSpeedBase ( 0,0.75 );
            //usleep(d);
        }

}
 void SpecificWorker::llegado(){
        differentialrobot_proxy->setSpeedBase ( 0,0 );
        target.setEmpty();
        estado=Estado::PARADO;
}

/**Funciones Jacobian**/
void SpecificWorker::goHome()
{
	RoboCompJointMotor::MotorStateMap mMap;
	try
	{
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap)
		{
			RoboCompJointMotor::MotorGoalPosition mg = { inner->getJoint(m.first)->home, 1.0, m.first };
			jointmotor_proxy->setPosition(mg);
		}
		sleep(1);
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}	
}



//////////////////
/// SLOTS
/////////////////

void SpecificWorker::leftSlot()
{
	error = QVec::vec6(INCREMENT,0,0,0,0,0);
}

void SpecificWorker::rightSlot()
{
	error = QVec::vec6(-INCREMENT,0,0,0,0,0);
}

void SpecificWorker::frontSlot()
{
	error = QVec::vec6(0,-INCREMENT,0,0,0,0);
}

void SpecificWorker::backSlot()
{
	error = QVec::vec6(0,INCREMENT,0,0,0,0);
}

void SpecificWorker::upSlot()
{
	error = QVec::vec6(0,0,INCREMENT,0,0,0);
}

void SpecificWorker::downSlot()
{
	error = QVec::vec6(0,0,-INCREMENT,0,0,0);
}

void SpecificWorker::changeSpeed(int s)
{
	FACTOR = s;
}

void SpecificWorker::moveArm()
{/*

	  
	  
	  else { 
	    closeHand();
	    sleep(3);	
	    //EJECUTAR LA VERRUGA
	    //CERRAR DEDOS Y BAJAR  _--> CREAR METODO PARA CERRAR  -> PARA ABRIR= GOHOME
	    
	    //msleep(500);
	    
	    qDebug()<< "Caja Cogida";
		for(auto m: joints)
		{
			RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
			vl.push_back(vg);
		}
	
	qDebug() << "Brazo Parado";
	qDebug() << "Llamar a coger caja";
 	goHome();
	
	  }

	//Do the thing
	try
	{ 
	  qDebug() << "7";
		jointmotor_proxy->setSyncVelocity(vl);
	}
	catch(const Ice::Exception &e)
	{	std::cout << "SetSyncVelocity "<<e.what() << std::endl;}
	 */
}
void SpecificWorker::cerrarMano(){
  	RoboCompJointMotor::MotorGoalPosition finger_right_1, finger_right_2;
	
	finger_right_1.name = "finger_right_1";
	finger_right_1.position = -0.6;
	finger_right_1.maxSpeed = 1;
	
	finger_right_2.name = "finger_right_2";
	finger_right_2.position = 0.6;
	finger_right_2.maxSpeed = 1;
	
	jointmotor_proxy->setPosition(finger_right_1);
	jointmotor_proxy->setPosition(finger_right_2);
}
