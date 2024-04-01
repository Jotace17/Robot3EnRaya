#include "TicTacToe.h"
#include <iostream>
#include <cmath>
#include <string>

int main()
{
    // Creating an object for the Game class
    Game* game = new Game;
    int ch;
    // Initializing a 2D array for the gameboard
    game->CreateBoard();
    // Select the mode of the game
    std::cout << "Press 1 for single-player and 2 for a two-player game" << std::endl;
    std::cin >> ch;
    // String to store the result from CheckWin() function
    char fin = ' ';
    // flag that indicates if the next player can play
    bool flag = true;
    int _dificultad = 0;
    // switch case for 2 modes of the game
    switch (ch)
    {
        // Singleplayer
    case 1:
    {
        // Creating object for player 1
        Player player("Player I");
        // iterate until a winner or draw is reached
        while (flag)
        {
            // Player turn
            game->PlayerTurn(player);
            // Check if player 1 has won the game or draw
            fin = game->CheckWin();
            if (fin != ' ')
            {
                goto point;
            }
            // Machine turn
            game->MachineTurn();
            // Check if any player won the game
            fin = game->CheckWin();
            // Conditions to check which player won the game or Draw
        point:
            if (fin == 'X')
            {
                std::cout << " X wins";
                // break the loop, as a result, is reached
                flag = false;
            }
            else if (fin == 'O')
            {
                std::cout << "O Wins";
                flag = false;
            }
            else if (fin == 'D')
            {
                std::cout << "The game ended in a draw";
                flag = false;
            }
        }
    }
    case 2:
    {
        // Creating two player objects
        Player player("X");
        Player player2("O");
        // iterate until a winner or draw is reached
        while (flag)
        {
            // player 1 turn
            game->PlayerTurn(player);
            // player 2 turn
            fin = game->CheckWin();
            if (fin != ' ')
            {
                goto point2;
            }
            game->PlayerTurn(player2);
            // Check if there is a winner or a draw in the match
            fin = game->CheckWin();
            // Conditions to check the winner or draw in the match
        point2:
            if (fin == 'X')
            {
                std::cout << " X wins";
                // break the loop, as a result, is reached
                flag = false;
            }
            else if (fin == 'O')
            {
                std::cout << "O Wins";
                flag = false;
            }
            else if (fin == 'D')
            {
                std::cout << "The game ended in a draw";
                flag = false;
            }
        }
    }
    // default case
    default:
    {
        exit(0);
    }
    }
    return 0;
}
