
#include "TicTacToe.h"

#include <iostream>
#include <cmath>
#include <string>
#include <time.h>

//#include "Arduino.h"

//int _dificultdad = 0;

void Game::CreateBoard()
{
    srand(time(NULL));
    // for loop to iterate over rows
    for (int i = 0; i < Dimension; i++)
    {
        // for loop to iterate over columns of the row
        for (int j = 0; j < Dimension; j++)
            //initialize empty value
            gameboard[i][j] = ' ';
    }
    // calling the showBoard() function.
    showBoard();
}

void Game::showBoard()
{
    printf("\n\n");
    // for loop to iterate over the rows
    for (int i = 0; i < Dimension; i++)
    {
        // leave space using the tab character
        std::cout << "\t\t\t";
        // iterate over the column of the row
        for (int j = 0; j < Dimension; j++)
            std::cout << " | " << gameboard[i][j];
        std::cout << " |\n\t\t\t----------------" << std::endl;
    }
}

void Game::PlayerTurn(Player& player)
{
    // position of a cell given by the player
    int position;
    // the row to which the position belongs
    int row;
    // Get the name of player using PlayerName() function
    std::cout << "Turn of " << player.PlayerName() << ":" << std::endl;
    // Get the position
    std::cout << "Enter the position to be marked: ";
    std::cin >> position;
    // compute the row using ceil() function
    row = ceil((float)position / Dimension);
    // compute the column to which the position belongs
    int col = (position - 1) % Dimension;
    // check condition to find if the position is empty
    if (gameboard[row - 1][col] != ' ')
    {
        std::cout << "Position already marked. Please select another position" << std::endl;
        // restart the function
        PlayerTurn(player);
    }
    else
    {
        // mark the position as X if it was player 1 
        // mark the position as Y if it was player 2
        player.PlayerName().compare("Player I") == 0 ? gameboard[row - 1][col] = 'X' : gameboard[row - 1][col] = 'O';
        std::cout << " Marked at position: " << position << std::endl;
        // increment count
        count++;
    }
    showBoard();
}

void Game::MachineTurn()
{
    int row, col;
    switch (_dificultad)
    {
    case ModoFacil:
        // Random position generated by rand() function
        int random_position;
        // computing a position between 1 and 9 for a 3X3 matrix
        std::cout << "Turn of Machine: " << std::endl;
        random_position = rand() % (Dimension * Dimension + 1);
        // computing the row in which the position is present
        row = ceil((float)random_position / Dimension);
        // computing the column in which the position is present
        col = (random_position - 1) % 3;
        //std::cout << " random " << random_position << std::endl;
        // Checking if the position is already marked by other symbols
        if (gameboard[row - 1][col] != ' ')
        {
            // call the function again to replay
            MachineTurn();

        }
        else
        {
            std::cout << " Marked at position: " << random_position << std::endl;
            // marking the position with O
            gameboard[row - 1][col] = 'O';
            // incrementing the count
            count++;
        }
        // calling the showBoard() function
        showBoard();
        break;

    case ModoMedio:
        break;

    case ModoDificil:
        break;

        //default:
            //break;
    }

}

/*
bool Game::CheckWin(Player player)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        // Check horizontals
        if (gameboard[i][0] == player && gameboard[i][1] == player && gameboard[i][2] == player)
            return true;

        // Check verticals
        if (gameboard[0][i] == player && gameboard[1][i] == player && gameboard[2][i] == player)
            return true;
    }

    // Check diagonals
    if (gameboard[0][0] == player && gameboard[1][1] == player && gameboard[2][2] == player)
        return true;

    if (gameboard[0][2] == player && gameboard[1][1] == player && gameboard[2][0] == player)
        return true;

    return false;
}
*/

char Game::CheckWin()
{
    char players[] = {'X', 'O'};

    for (const char& player : players)
    {
        for (unsigned int i = 0; i < 3; i++)
        {
            // Check horizontals
            if (gameboard[i][0] == player && gameboard[i][1] == player && gameboard[i][2] == player)
                return player;

            // Check verticals
            if (gameboard[0][i] == player && gameboard[1][i] == player && gameboard[2][i] == player)
                return player;
        }

        // Check diagonals
        if (gameboard[0][0] == player && gameboard[1][1] == player && gameboard[2][2] == player)
            return player;

        if (gameboard[0][2] == player && gameboard[1][1] == player && gameboard[2][0] == player)
            return player;

    }

    if (count == Dimension * Dimension)
    {
        return 'D';
    }
    return ' ';
}

    

/* //ORIGINAL
std::string Game::CheckWin()
{
    // variable used to check in the horizontal direction
    int r1 = 0, r2 = 0;
    // variable used to check in the vertical direction
    int c1 = 0, c2 = 0;
    // variable used to check in the diagonal direction
    int d1 = 0, d2 = 0;
    // symbol X and O
    char x = 'X';
    char O = 'O';
    // iterate over the rows
    for (int i = 0; i < Dimension; i++)
    {
        // initialize horizontal and vertical variables to zero
        r1 = 0, r2 = 0;
        c1 = 0, c2 = 0;
        // Increment for diagonal values on checking consecutive diagonal values.
        if (gameboard[i][i] == x)
        {
            d1++;
            //count++;
        }
        else if (gameboard[i][i] == O)
        {
            d2++;
            //count++;
        }
        // iterate the columns
        for (int j = 0; j < Dimension; j++)
        {
            // Increment for horizontal values on checking consecutive rows
            if (gameboard[i][j] == x)
            {
                r1++;
            }
            else if (gameboard[i][j] == O)
            {
                r2++;
            }
            // Increment for vertical values on checking consecutive columns
            if (gameboard[j][i] == x)
            {
                c1++;
            }
            else if (gameboard[j][i] == O)
            {
                c2++;
            }
        }
        // A player won by matching horizontally
        if (r1 == Dimension || r2 == Dimension)
        {
            return (r1 == Dimension) ? "Player I" : "Player 2";
        }
        // A player won by matching vertically
        else if (c1 == Dimension || c2 == Dimension)
        {
            return (c1 == Dimension) ? "Player I" : "Player 2";
        }
        // A player won by matching diagonally
        else if (d1 == Dimension || d2 == Dimension)
        {
            return (d1 == Dimension) ? "Player I" : "Player 2";
        }
    }
    if (count == Dimension * Dimension)
    {
        return "Draw";
    }
    return "";
}
*/