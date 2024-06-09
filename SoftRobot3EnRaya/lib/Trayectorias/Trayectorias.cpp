
#include "trayectorias.h"

#include <iostream>
#include <cmath>
#include <vector>

// DEFINICIÓN DE CONSTANTES

#define L_ARM1 300
#define L_ARM2 210

#define X_A 0
#define Y_A 0

using namespace std;

// Primera función : Cinematica Directa

// La función retorna la posición en forma de vector

// Como parámetros de entrada contiene los angulos de la base, el angulo del brazo, y el angulo del antebrazo

vector<float> Kinet_Dir(const float q1, const float q2, const float q3)
{
    vector<float> Pos;

    float r;

    float x, y, z;

    r = L_ARM1*cos(q2) + L_ARM2*cos(q3-q2);

    cout << "La distancia r es : " << r << endl;

    x = r*sin(q1);
    y = r*cos(q1);
    z = L_ARM1*sin(q2) + L_ARM2*sin(q2-q3);

    Pos = {x, y, z};

    cout << "El valor de x es : " << x << endl;
    cout << "El valor de y es : " << y << endl;
    cout << "El valor de z es : " << z << endl;

    return Pos;
}// Fin de la funcion Kinet_Dir()



// Segunda Función : Cinemática Inversa

// La función devuelve los ángulos de cada uno de los eslabones, además de la base

// Como parámetros de entrada se le pasan la posición del extremo del brazo

vector<float> Kinet_Inver(const float posx, const float posy, const float posz)
{
    vector<float> angle;

    float q0 = 0, q1 = 0, q2 = 0;

    // Primero hallamos el angulo de la base

    q0 = atan2(posx,posy);


    // Hallamos el ángulo de q2, correspondiente al angulo del antebrazo

    // -----------------------------------------------------------------

    // Hallamos "r"

    float r = sqrt(pow(posx,2) + pow(posy,2));

    float lg = sqrt(pow(posz,2) + pow(r,2));

    float beta = acos((pow(L_ARM1,2) + pow(L_ARM2,2) - pow(r,2) - pow(posz,2))/(2*L_ARM1*L_ARM2));

    q2 = M_PI - beta;

    // --------------------------------------------------------------------------------------------

    // Por último hallamos el valor del angulo q1, este se corresponde al angulo del brazo

    // cout << "-------------------------------------" << endl;

    // cout << "El valor de r es : " << r << endl;

    // cout << "EL valor de lg es : " << lg << endl;

    float thau = acos(r/lg);

    // cout << "El valor de thau es :" << thau*180/M_PI << endl;

    float gamma = acos((pow(L_ARM2,2) + pow(lg,2) - pow(L_ARM1,2))/(2*L_ARM2*lg));

    // cout << "El valor de gamma es :" << gamma*180/M_PI << endl;

    float alpha = M_PI - gamma - beta;

    // cout << " El valor de alpha es :" << alpha*180/M_PI << endl;

    q1 = thau + alpha;

    // cout << "El valor de beta es :" << beta*180/M_PI << endl;

    // cout << "-------------------------------" << endl;

    // Retornamos los valores de los ángulos

    angle = {q0, q1, q2};

    return angle;
}

// int main()
// {
//     cout << "Hello World!" << endl;

//     vector<float> Pos = Kinet_Dir(60*M_PI/180, 110*M_PI/180, 80*M_PI/180);

//     vector<float> q;

//     q = Kinet_Inver(Pos[0], Pos[1], Pos[2]);

//     for (int i = 0; i<3; i++)
//     {
//         cout << q[i]*180/M_PI << endl;
//     }

//     return 0;

// }



