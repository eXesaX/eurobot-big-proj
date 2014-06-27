/* PathFinder
*  library specifically for URFU-2 on Eurobot
*
*  License: WTFPL v2 (http://www.wtfpl.net/txt/copying/)
*/
#ifndef PathFinder_h
#define PathFinder_h

#include "Arduino.h"
#include <QueueList.h>
#include <math.h>

#define DELTA 10

typedef struct{
    int x1;
    int y1;
    int x2;
    int y2;
} Obstacle;

typedef struct{
    int x;
    int y;
} Position;

typedef struct{
    Position in;
    Position out;
    char obj_i;
} Cross;

class PathFinder{
	public:
		PathFinder();
		void AddObject(Obstacle new_object);
		void SetPosition(int x, int y, int degree);
		void ChangePosition(int x, int y, int degree);
		void AddTempObject();
		void RemoveTempObject();
		void FindPath(int x, int y, Position *out);
	private:
		Obstacle objects[20];
		Position active_position;
		int active_degree;
		bool have_temp_object;
		unsigned char objects_count;
		void FindPath(int x, int y, int from_x, int from_y, Position *out);
                char calc_sector(int delta_x, int delta_y);
                Cross check_one_side(char i, char sector, int target_x, int target_y, int from_x, int from_y);
                Cross check_two_side(char i, char sector, int target_x, int target_y, double k, double b);
                float calc_line(int x, int y, double k, int b);
                unsigned int get_distance(int *robot_x, int *robot_y, int *target_x, int *target_y);
                void get_next_direction(Position *next_path, Cross *object, char sector);
                char get_type_direction(Cross *cross, char i, char sector);
                bool check_line(Position start, Position finish);
};

#endif /* PathFinder_h */

