#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <iostream>

int z = 100, x = 0, zcent = 90;
int xmin = -800, xmax = 800;  // -adelante +reversa  :MAX hardware! [-2500, 2500]
int zmin = 20, zmax = 160;  // izq=180, centro = 90, der=0
bool reverza = false;
int id_stering = 0; // 0 o 3
bool first_run = true;

/* 360 control
left joy = 1, right joy = -1
RT = 1, full = -1
buton = 0, push = 1
*/

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  // Dirección del carro
  if (joy->axes[id_stering] > 0.2){  // Izq
    z = (int) (90 + joy->axes[id_stering] * 90) ;
  }
  else if (joy->axes[id_stering] < -0.2){  // Der
    z = (int) (90 + joy->axes[id_stering] * 90)  ;
  }
  else{//Neutro
    z = zcent;
  }
  
  // Sentido de avance
  if (joy->buttons[3]){  // Y
    reverza = false;
  }
  if (joy->buttons[1]) {  // B
    reverza = true;
  } 

  // Acelerar-reverza
  if(!reverza){   // Adelante

    if (joy->axes[5] < 0.9){  // Acelerar

      if (x > 0){  // si va en reversa
        x -= 20;  // decrementa velocidad

      }else{ // avanza a escala del valor máximo
        x = -xmin * ((joy->axes[5] - 1) * (0.5));

      }
    }else if(x < 0){ // Neutro
      x += 20;

      if(x > 0){
        x = 0;
      }
    }
  }else{ // atras

    if (joy->axes[5] < 0.9){  // Acelerar

      if (x < 0){  // si va hacia delante
        x += 20;  // aumenta velocidad

      }else{ // avanza a escala del valor máximo
        x = xmax * ((joy->axes[5] - 1) * (-0.5));

      }
    }else if(x > 0){ // Neutro
      x -= 20;

      if(x < 0){
        x = 0;
      }
    }
  }


  if(joy->buttons[4] || joy->buttons[5]){  // Paro de emergencia 
    first_run = false;
    x = 0;
    z = zcent;
  }

  // Ajuste de Límites
  if (x > xmax) 
    x = xmax;
  if (x < xmin) 
    x = xmin;
  if (z > zmax)
    z = zmax;
  if (z < zmin)
    z = zmin;

  
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "JoyControl");
   ros::NodeHandle nh_;
   ros::Subscriber information_joy = nh_.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
   ros::Publisher speedpub_ = nh_.advertise<std_msgs::Int16>("/manual_control/speed", 1);
   ros::Publisher steeringpub_ = nh_.advertise<std_msgs::Int16>("/manual_control/steering", 1);
   std_msgs::Int16 speed;
   std_msgs::Int16 steering;
   ros::Rate loop_rate(30);//10
   
   while(ros::ok()) {
      speed.data = x;
      speedpub_.publish(speed);
      steering.data = z;
      steeringpub_.publish(steering);
      ros::spinOnce();
      loop_rate.sleep();
   }
}