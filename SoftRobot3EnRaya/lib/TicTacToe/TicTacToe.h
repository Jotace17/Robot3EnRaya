#pragma once

//Se pone parte declarativa
#include <iostream>
#include "Keypad.h"


#define Dimension 3
#define ModoFacil false
#define ModoDificil true


extern Keypad keypad; 
extern const int pinTurnButton;
extern int turnButtonState;


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
    bool ChangeDificulty();

};
