#include "Robot.h"
#include "mbed.h"

// ==== Math Constants ==== //
#define M_PI 3.14159265358979323846

// ==== Bot Constants ==== //
#define ep 1440 // Number of Encoder Pulses
#define radi 3.5 // Raio das Rodas
#define b 14 //Distancia entre Rodas
#define wmax 150 // Velocidade máxima


//int16_t countsLeft,countsRight = 0.0;
double D_left,D_right,Delta_D,Delta_teta=0.0;
double x,y,phi_c;
double we_abs,wd_abs=0.0;

// Filtro Velocidades
void v_filter(double *we,double *wd){
    we_abs = fabs(*we);
    wd_abs = fabs(*wd);
    if(we_abs > wmax || wd_abs > wmax){
        if(we_abs > wd_abs){
            *we = wmax;
            *wd = wmax*wd_abs/we_abs;
        }
        else if(we_abs < wd_abs){
            *wd = wmax;
            *we = wmax*we_abs/wd_abs;
        }
        else if (we_abs == wd_abs){
            *we = wmax;
            *wd = wmax;
        }
    }
}

// Odometria
void odometria(){
    D_left=(double)(countsLeft*2*M_PI*radi)/ep;
    D_right=(double)(countsRight*2*M_PI*radi)/ep;
    Delta_D = (double)(D_left+D_right)/2; // Deslocamento do Centro de Massa (CG)
    Delta_teta=(double)(D_right-D_left)/b;
}

// Estimar Posição
bool estimate_pos(){
    if(Delta_teta==0){ // No Ponto Nao Definido (Objective)
        // x(k+1)=x(k)+Delta_D*cos(phi(k))
        
        x += Delta_D*cos(phi_c+(Delta_teta/2));
        y += Delta_D*sin(phi_c+(Delta_teta/2));
        //led = !led; // Im on Objective
        return true;
    }
    else{
        x += Delta_D*(sin(Delta_teta/2)/(Delta_teta/2))*cos(phi_c+Delta_teta/2);
        y += Delta_D*(sin(Delta_teta/2)/(Delta_teta/2))*sin(phi_c+Delta_teta/2);
        phi_c += Delta_teta;
        return false;
    }
}
