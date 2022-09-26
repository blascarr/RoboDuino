

 #ifndef Robo2Duino_h
    #define Robo2Duino_h	
    // --------------------------------- //
	// -------- Pose 2D Class -----------//
 	 
	Pose2D se2(float px , float py , float angle, bool inv = 1 ){
		float transM[3][3] = {{cos(angle/ang2rad),-sin(angle/ang2rad),px},{sin(angle/ang2rad),cos(angle/ang2rad),py},{0, 0,1}};
		Pose2D m;
		m = transM;
		return m;
	}

 	rot2D rot2(float angle){
		float arrayRot[2][2] = {{cos(angle/ang2rad),-sin(angle/ang2rad)},{sin(angle/ang2rad),cos(angle/ang2rad)}};
		rot2D m;
		m = arrayRot;
		return m;
	}

	Pose2D trot2( float angle ){
		Pose2D m = se2 (0, 0, angle);
		return m;	
	}

	Pose2D transl2( float px , float py ){
		Pose2D m = se2 (px, py, 0);
		return m;
	}

	Pose2D transl2( vector2D p ){
		Pose2D m = se2 (p(0,0), p(0,1), 0);
		return m;
	}

	class Bot2DController {
		public:
			Pose2D T;
			Bot2DController(void){
				Bot2DController::T = se2(0,0,0);
			};

			Bot2DController(float px , float py , float angle){
				Bot2DController::T = se2(px, py, angle);
			};

			Bot2DController(Bot2DController *p){
				Bot2DController::T = p->T;
			};

			void set(float px , float py , float angle){
				Bot2DController::T = se2(px, py, angle);
			};

			Pose2D Bot2DController::move( float px , float py ){
				Pose2D m = se2(px, py, 0);
				Bot2DController::T *= m;
				return Bot2DController::T;
			};

			Pose2D Bot2DController::move( Pose2D m ){
				Bot2DController::T = Bot2DController::T*m;
				return Bot2DController::T;
			};

			Bot2DController::print(){
				Serial.print(" Pose :  ");
				Serial << Bot2DController::T << "\n";
			}

			Pose2D turn(float angle){
				Pose2D m = se2(0, 0, angle);
				Bot2DController::T *= m;
				return Bot2DController::T;
			};

	};

	class Bot2D : public Bot2DController {
		public:

			Bot2D(void) : Bot2DController( ){
				
			};

			Pose2D fwd(float d){
				Bot2D::move(d,0);
				return Bot2D::T;
			};

			Pose2D back(float d){
				Bot2D::move(-d,0);
				return Bot2D::T;
			};

			Pose2D left(float d){
				Bot2D::move(0,d);
				return Bot2D::T;
			};

			Pose2D right(float d){
				Bot2D::move(0,-d);
				return Bot2D::T;
			};
	};

	class Link2D : public Bot2DController{
		public:
			//Pose2D[] l;

	};

	// -------- Point 2D Class -----------//
	class Point2D{
		public:
			vector2D p;
			ref2D ref;
			
			Point2D(float x0 = 0, float y0 = 0) {
				p(0) = x0;
				p(1) = y0;
			};
			void setX( float x ) { Point2D::p(0) = x; }
			uint16_t getX(){ return p(0); }
			void setY( float y ) { Point2D::p(1) = y; }
			uint16_t getY(){ return p(1) ; }
			void setXY( float x, float y ) { setX( x ); setY( y ); }
			
			void move(float dx, float dy){
				p(0) += ref.i*dx;
				p(1) += ref.j*dy;
			};

			void move( Pose2D m ){
				move(m(0,2), m(1,2));
			}
			void fwd( float d ){
				move(d,0);
			};
			void back( float d ){
				move(-d,0);
			};
			void left( float d ){
				move(0,d);
			};
			void right( float d ){
				move(0, -d);
			};
	};

	class Pixel2D : public Point2D{
		public:
			uint16_t color;
			Pixel2D(float x0 = 0, float y0 = 0, uint16_t colour = 0 ): Point2D( x0, y0 ), color( colour ){};
			void setColor( uint16_t colour ) { color = colour; }
			uint16_t getColor(){ return color; }
	};

 	Point2D Mt2Pt ( Pose2D m ){
		return Point2D (m(0, 2), m(1, 2));
	}

	Pose2D Pt2Mt ( Point2D P ){
		return se2( P.p(0,2) , P.p(1,2) , 0 );
	} 
	
	float distance(float x0, float y0, float xf, float yf){
		return sqrt(pow(xf-x0,2)+pow(yf-y0,2));
	}

	float distance(Point2D p0, Point2D pf){
		return sqrt(pow(pf.p(0,0)-p0.p(0,0),2)+pow(pf.p(0,1)-p0.p(0,1),2));
	}

	float distance(Pose2D P0, Pose2D Pf){
		return sqrt(pow(Pf(0,2) - P0(0,2),2)+pow(Pf(1,2) - P0(1,2),2));
	}

#endif