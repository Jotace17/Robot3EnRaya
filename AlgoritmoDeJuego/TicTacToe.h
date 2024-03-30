#pragma once
//Se pone parte declarativa
#include <iostream>

/*
#include <ESP32Servo.h>
extern Servo _servo;

#define PIN_POT_IN 4
#define PIN_SERVO 12

void InitPot();
void InitServo();

float GetAngPot_Deg();
void MoveServo(float Ang);
*/

#define Dimension 3

#define ModoFacil 0
#define ModoMedio 1
#define ModoDificil 2

//int _row, _col, _pos;

//int _dificultad = 0;


class Player
{
private:
    std::string name;
public:
    // constructor
    Player(std::string n)
    {
        name = n;
    }
    std::string PlayerName()
    {
        return this->name;
    }
};


class Game
{
private:

    char gameboard[Dimension][Dimension];
    int count = 0;
    int _dificultad = 0;

public:

    // CreateBoard Function
    void CreateBoard();

    // showBoard() function
    void showBoard();

    // PlayTurn function with player object as parameter
    void PlayerTurn(Player& player);

    // function MachineTurn
    void MachineTurn();

    int RandomPos();
    char CheckWin();

    // function CheckWin()
    //std::string CheckWin();

};

