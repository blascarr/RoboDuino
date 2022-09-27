#include <RoboDuino.h>

float arr[3]={6.54, 3.66, 2.95};
Pose2D p;
Pose2D X = transl2(2,3);
Pose2D R = trot2(2/rad2ang);
vector2D C = {1,2}; 
Pose2D RC = transl2(C)*R*transl2(-C);

void setup() {
  Serial.begin(9600);

  p = se2( 5 , 5 , 30);
  Serial.println();
  
  Serial << "Transl2( 2,3) " << X ;
  Serial.println();
  Serial << "Trot2( 2) " << R;
  Serial.println();
  Serial << "R*X " << R*X;
  Serial.println();
  Serial << "X*R " << X*R;
  Serial.println();

  Serial << "C " << C;
  Serial.println();
  Serial << "RC " << RC;
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:

}
