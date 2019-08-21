#include <iostream>

using namespace std;

int main() {

    char OR = 'N', AC[100];   // = 'S', actions[100];        // OR mean orientation (N/E/W/S), AC means action taken (L or R)
    int dists[100], n;      //dists for distances and n for number of actions
    cout << "Enter how many actions is the bot taking, then enter actions after spaces \n and after that enter the distances between those actions \n";
    cin >> n;

    cout << "Now enter only actions: \n";
    for (int i = 0; i < n; i++)
    {
        cin >> AC[i];


        /* code */
    }

    /*for (int i = 0; i < n; i++)
    {
        cin >> dists[i];


        //WILL ADD THIS PART LATER, DID NOT CONSIDER NECESSARY FOR JUST A LOW LEVEL TESTING CODE.
    } */

    cout << "Now the bot is oriented in the: ";

    int operation = 0;
    char currentOR = 'N', prevOR = 'N';         // currentOR = current orientation and prevOR = previous orientation

    for (int i = 0; i < n; i++)
    {
        operation = 0;
        if (prevOR == 'N')
        {
            if (AC[i] == 'L')
            {
                currentOR = 'W';
                prevOR = currentOR;
                operation=1;
            }

            else if (AC[i] == 'R' && operation<1)
            {
               currentOR = 'E'; 
               prevOR = currentOR;
                operation=1;
            }
            
        }

        else if (prevOR == 'E' && operation<1)
        {
            if (AC[i] == 'L')
            {
                currentOR = 'N';
                prevOR = currentOR;
                operation=1;
            }

            else if (AC[i] == 'R' && operation<1)
            {
                currentOR = 'S';
                prevOR = currentOR;
                operation=1;
            }


        }

        else if (prevOR == 'W' && operation<1)
        {
           if (AC[i] == 'L')
            {
                currentOR = 'S';
                prevOR = currentOR;
                operation=1;
            }

            else if (AC[i] == 'R')
            {
                currentOR = 'N';
                prevOR = currentOR;
                operation=1;
            } 
        }

        else if (prevOR == 'S' && operation<1)
        {
            if (AC[i] == 'L')
            {
                currentOR = 'E';
                prevOR = currentOR;
                operation=1;
            }

            else if (AC[i] == 'R')
            {
                currentOR = 'W';
                prevOR = currentOR;
                operation=1;
            }

        }

        //
        
        
    }

    cout << currentOR << " direction \n";
    
    
    


 
    return 0;
}