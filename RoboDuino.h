/* RoboDuino
  Design and created by Blascarr

  RoboDuino
  Name    : Blascarr
  Description: RoboDuino.h
  version : 1.0

RoboDuino is a generic library useful for Advanced Robotics Applications.

This library is part of a educational course to learn C++ in practice in https://github.com/blascarr/RoboDuino Couser or http://www.blascarr.com/ webpage.

 *	
 *  This Library gives some helpful methods to manipulate transformation Matrix in order to create references
 *	With RoboDuino we can integrate different methods based on Matrix with BasicLinearAlgebra library
 *
 *	https://github.com/tomstewart89/BasicLinearAlgebra
 *
 *  Useful for Robotic Arms and TFT Screen https://github.com/blascarr/TFTCourse
 *Written by Adrian for Blascarr
 */

 #ifndef RoboDuino_h
    #define RoboDuino_h	
    #include "BasicLinearAlgebra.h"
	#include <stdarg.h>
	#include <Geometry.h>
    
	#if defined(ARDUINO) && ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
	#endif
 	using namespace BLA;

    float ang2rad=180/PI;
	float rad2ang=PI/180;

	typedef Matrix< 1, 2 > vector2D;
	typedef Matrix< 1, 3 > vector3D;
	typedef Matrix< 2, 2 > rot2D;
	typedef Matrix< 3, 3 > Pose2D;
	typedef Matrix< 4, 4 > Pose3D;

    struct ref2D{   
		bool swerve = true;     // Levorotation = true ( z-axis - up normal on paper ) , Dextrorotation = false
		int i = 1;
		int j = 1;               // Unitary vector x, y direction
		ref2D( bool swerve = true ): swerve( swerve ){
			if( !swerve ){j = -1;}
		}
	} ;

    #include "src/Robo2Duino.h"
    #include "src/Robo3Duino.h"

#endif