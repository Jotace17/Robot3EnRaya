
#include "TicTacToe.h"
#include "Keypad.h"

#include <iostream>
#include <cmath>
#include <string>
#include <time.h>

int _row, _col, _pos;
bool _dificultad = false;

int charToInt(char key);

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
    //showBoard();
}

void Game::showBoard()
{
    Serial.printf("\n\n");
    // for loop to iterate over the rows
    for (int i = 0; i < Dimension; i++)
    {
        // leave space using the tab character
        Serial.print("\t\t\t");
        // iterate over the column of the row
        for (int j = 0; j < Dimension; j++)
            Serial.printf(" | %c", gameboard[i][j]);
        Serial.printf(" |\n\t\t\t----------------\n");
    }
}

void Game::PlayerTurn(Player& player)
{
    // position of a cell given by the player
    int position = -1;
    // the row to which the position belongs
    int row, col;

    // Get the name of player using PlayerName() function
    std::cout << "Turn of " << player.PlayerName() << ", choose a cell:" << std::endl;

    int pos_temp = -1;
    while(turnButtonState == LOW || position == -1)
    {
        char key = keypad.getKey();
        if(key)
        {
            pos_temp = charToInt(key);
        }
        if(pos_temp != -1)
        {
            position = pos_temp;
            //pos_temp = -1;
        }
        turnButtonState = digitalRead(pinTurnButton);
        delay(20);

    }
    turnButtonState = LOW;

    // compute the row using ceil() function
    row = ceil((float)position / Dimension);
    // compute the column to which the position belongs
    col = (position - 1) % Dimension;

    // check condition to find if the position is empty
    if (gameboard[row - 1][col] != ' ')
    {
        Serial.printf("Position already marked. Please select another position \n");

        // restart the function
        PlayerTurn(player);
    }
    else
    {
        // mark the position as X if it was player 1 
        // mark the position as Y if it was player 2
        player.PlayerName().compare("Player X") == 0 ? gameboard[row - 1][col] = 'X' : gameboard[row - 1][col] = 'O';
        Serial.printf("Marked at position: %d \n", position);
        // increment count
        count++;
    }
    showBoard();
}

void Game::MachineTurn()
{
    switch (_dificultad)
    {
    case ModoFacil:
        Serial.printf("Modo dificil: %d \n" , _dificultad);
        RandomPos();

        if (gameboard[_row - 1][_col] != ' ')
        {
            // call the function again to replay
            MachineTurn();
        }
        else
        {
            Serial.printf("Marked at position: %d\n", _pos);
            // marking the position with O
            gameboard[_row - 1][_col] = 'O';
            // incrementing the count
            count++;
        }
        // calling the showBoard() function
        showBoard();
        break;

    case ModoDificil:
        Serial.println("mamaste");
        break;
    }

}

bool Game::ChangeDificulty()
{
    if(_dificultad)
        {
            _dificultad = false;
            Serial.printf("Dificultad: %s \n", "Facil");
        }else 
        {
            _dificultad = true;
            Serial.printf("Dificultad: %s \n", "Dificil");
        } 
   return _dificultad;
}

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

int Game::RandomPos()
{
    // Random position generated by rand() function
    int random_position; // , row, col;
    // computing a position between 1 and 9 for a 3X3 matrix
    Serial.printf("Turn of Machine: \n");

    random_position = rand() % (Dimension * Dimension + 1);
    // computing the row in which the position is present
    _row = ceil((float)random_position / Dimension);
    // computing the column in which the position is present
    _col = (random_position - 1) % 3;
    _pos = random_position;
    Serial.printf("Random pos: col %d, row %d  \n",_col, _row);

    //return row, col, random_position;
    return 0;
}

int charToInt(char key) {
    // Asegurarse de que el carácter es un dígito
    if (key >= '0' && key <= '9') {
        return key - '0';
    } else {
        // Manejar el caso en que el carácter no es un dígito válido
        return -1;  // Valor de error o manejarlo como prefieras
    }
}