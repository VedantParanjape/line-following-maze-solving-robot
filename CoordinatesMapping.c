#include <iostream>

using namespace std;

int juncNo = 0;

int main( )
{
    int arr[10][10], x=0 , y=0 ;    //x, y co-ordinates and the i is junc number and j is junc. type no.
    while(junction_detected == 1)
    {

        if(bot_facing == 'N' )
        {
            y += encoder_value;      //how to increase the value stored in i explicitly?
             
        
        }
        else if( bot_facing == 'S')
        {
            y -= encoder_value;
        }
        else if( bot_facing == 'E')
        {
            x += encoder_value;
        }
        else if( bot_facing == 'W')
        {
            x -= encoder_value;
        }

        store_junction_location(i, j, junctionType);

    }


}

void store_junction_location(int x, int y, int junctionType)
{
    int coordinates[juncNo][junctionType] = { x, y };  
    juncNo++;
}