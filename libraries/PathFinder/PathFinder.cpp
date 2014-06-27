/* PathFinder
*  library specifically for URFU-2 on Eurobot
*
*  License: WTFPL v2 (http://www.wtfpl.net/txt/copying/)
*/

#include "PathFinder.h"

PathFinder::PathFinder(){
    objects_count = 0;
    have_temp_object = false;
}

void PathFinder::AddObject(Obstacle new_object){
    objects[objects_count++] = new_object;
}

void PathFinder::SetPosition(int x, int y, int degree){
    active_position.x = x;
    active_position.y = y;
    active_degree = degree;
}

void PathFinder::ChangePosition(int x, int y, int degree){
    active_position.x += x;
    active_position.y += y;
    active_degree += degree;
}

void PathFinder::AddTempObject(){

}

void PathFinder::RemoveTempObject(){

}

void PathFinder::FindPath(int x, int y, Position *out){
    FindPath(x, y, active_position.x, active_position.y, out);
}

void PathFinder::FindPath(int x, int y, int from_x, int from_y, Position *out){
    char sector = calc_sector(from_x - x, from_y - y);
    unsigned int distance = 0;
    double k;
    double b;
    Cross active_object;
    if(sector < 4){
        k = (y - from_y) / (x - from_x);
        b = (k*x + y) * -1;
    }
    for(char i=0; i<objects_count; ++i){
        Cross collision_point;
        if(sector < 4){
            collision_point = check_two_side(i, sector, x, y, k, b);
        } else {
            collision_point = check_one_side(i, sector, x, y, from_x, from_y);
        }

        if(collision_point.in.x != -1 && collision_point.in.y != -1){
            unsigned int temp_d = get_distance(&x, &y, &(collision_point.in.x), &(collision_point.in.y));
            if(temp_d < distance || distance == 0){
                distance = temp_d;
                active_object = collision_point;
            }
        } else {
            continue;
        }
    }

    if(distance == 0)
        *out = (Position){x, y};
    else {
        get_next_direction(out, &active_object, sector); //TODO
    }
}

char PathFinder::calc_sector(int delta_x, int delta_y){
    if(delta_x == 0 && delta_y == 0){
        return -1;
    }
    if(delta_x > 0 && delta_y > 0)
        return 0;
    else if(delta_x > 0 && delta_y < 0)
        return 1;
    else if(delta_x < 0 && delta_y < 0)
        return 2;
    else if(delta_x < 0 && delta_y > 0)
        return 3;
    else if(delta_x == 0 && delta_y > 0)
        return 4;
    else if(delta_x > 0 && delta_y == 0)
        return 5;
    else if(delta_x == 0 && delta_y < 0)
        return 6;
    else
        return 7;
}

Cross PathFinder::check_one_side(char i, char sector, int target_x, int target_y, int from_x, int from_y){
    switch(sector){
    case 4 : if(objects[i].x1 >= target_x && objects[i].x2 <= target_x && objects[i].y1 <= target_y && from_y <= objects[i].y1){
                 Cross temp;
                 temp.in = (Position){target_x, objects[i].y1};
                 temp.out = (Position){target_x, objects[i].y2};
                 temp.obj_i = i;
                 return temp;
             }
             break;
    case 6 : if(objects[i].x1 >= target_x && objects[i].x2 <= target_x && objects[i].y2 >= target_y && from_y >= objects[i].y2){
                 Cross temp;
                 temp.in = (Position){target_x, objects[i].y2};
                 temp.out = (Position){target_x, objects[i].y1};
                 temp.obj_i = i;
                 return temp;
             }
             break;
    case 5 : if(objects[i].y1 >= target_y && objects[i].y2 <= target_y && objects[i].x2 <= target_x && from_x <= objects[i].x2){
                 Cross temp;
                 temp.in = (Position){objects[i].x2, target_y};
                 temp.out = (Position){objects[i].x1, target_y};
                 temp.obj_i = i;
                 return temp;
             }
             break;
    case 7 : if(objects[i].y1 >= target_y && objects[i].y2 <= target_y && objects[i].x1 >= target_x && from_x >= objects[i].x1){
                 Cross temp;
                 temp.in = (Position){objects[i].x1, target_y};
                 temp.out = (Position){objects[i].x2, target_y};
                 temp.obj_i = i;
                 return temp;
             }
             break;
    }
    Cross temp;
    temp.in = (Position){-1, -1};
    temp.out = (Position){-1, -1};
    temp.obj_i = i;
    return temp;
}

Cross PathFinder::check_two_side(char i, char sector, int target_x, int target_y, double k, double b){
    int *x_in;
    int *y_in;
    int *x_out;
    int *y_out;

    if (sector == 0 || sector == 1){
        x_in = &(objects[i].x2);
        x_out = &(objects[i].x1);
    } else {
        x_in = &(objects[i].x1);
        x_out = &(objects[i].x2);
    }

    if (sector == 0 || sector == 3){
        y_in = &(objects[i].y1);
        y_out = &(objects[i].y2);
    } else {
        y_in = &(objects[i].y2);
        y_out = &(objects[i].y1);
    }
    Cross out;
    out.obj_i = i;

    int calc_temp = calc_line(*x_in, -1, k, b);
    if(objects[i].y1 >= calc_temp && calc_temp <= objects[i].y2){
        out.in = (Position){*x_in, calc_temp};
    } else {
        calc_temp = calc_line(-1, *y_in, k, b);
        if(objects[i].x2 >= calc_temp && calc_temp <= objects[i].x1){
            out.in = (Position){calc_temp, *y_in};
        } else {
            return (Cross){(Position){-1, -1},(Position){-1, -1}, i};
        }
    }

    calc_temp = calc_line(*x_out, -1, k, b);
    if(objects[i].y1 >= calc_temp && calc_temp <= objects[i].y2){
        out.out = (Position){*x_out, calc_temp};
    } else {
        calc_temp = calc_line(-1, *y_out, k, b);
        if(objects[i].x2 >= calc_temp && calc_temp <= objects[i].x1){
            out.out = (Position){calc_temp, *y_out};
        } else {
            //Во те и поворот! Вектор внутрь фигуры входит, но нигде не выходит! т.е. как бы такого быть не может.
        }
    }

    return out;
}

float PathFinder::calc_line(int x, int y, double k, int b){
    if(x == -1){
        return (y - b) / k;
    } else if(y == -1){
        return k*x + b;
    }
    return 1;
}

unsigned int PathFinder::get_distance(int *robot_x, int *robot_y, int *target_x, int *target_y){
	return sqrt(pow((long) *target_x - *robot_x, 2) + pow((long) *target_y - *robot_y, 2));
}

void PathFinder::get_next_direction(Position *next_path, Cross *object, char sector){
    //Position next_path[3];

    if(sector == 4){

        next_path[0] = (Position){object->in.x, object->in.y - DELTA};
        if(objects[object->obj_i].x1 - object->in.x >= object->in.x - objects[object->obj_i].x2){
            //Вниз
            next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
        } else {
            //Вверх
            next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
        }
        next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y2 + DELTA};

    } else if(sector == 5){

        next_path[0] = (Position){object->in.x - DELTA, object->in.y};
        if(objects[object->obj_i].y2 - object->in.y >= object->in.y - objects[object->obj_i].y1){
            //Влево
            next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
        } else {
            //Вправо
            next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
        }
        next_path[2] = (Position){objects[object->obj_i].x1 + DELTA, next_path[1].y};

    } else if(sector == 6){

        next_path[0] = (Position){object->in.x, object->in.y + DELTA};
        if(objects[object->obj_i].x1 - object->in.x >= object->in.x - objects[object->obj_i].x2){
            //Вниз
            next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
        } else {
            //Вверх
            next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
        }
        next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y1 - DELTA};

    } else if(sector == 7){

        next_path[0] = (Position){object->in.x + DELTA, object->in.y};
        if(objects[object->obj_i].y2 - object->in.y >= object->in.y - objects[object->obj_i].y1){
            //Влево
            next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
        } else {
            //Вправо
            next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
        }
        next_path[2] = (Position){objects[object->obj_i].x2 - DELTA, next_path[1].y};

    } else if(sector == 0){

        if(get_type_direction(object, object->obj_i, sector) == 2){
            if(objects[object->obj_i].x2 == object->in.x){
                char dir;
                //Снизу вверх
                next_path[0] = (Position){object->in.x - DELTA, object->in.y};
                if(object->in.y <= objects[object->obj_i].y1 + (objects[object->obj_i].y1 + objects[object->obj_i].y2)/2){
                    // По часовой стрелке
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x1 + DELTA, next_path[1].y};
                } else {
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x1 + DELTA, next_path[1].y};
                }
            } else {
                char dir;
                //Слева направо
                next_path[0] = (Position){object->in.x, object->in.y - DELTA};
                if(object->in.x <= objects[object->obj_i].x1 + (objects[object->obj_i].x1 + objects[object->obj_i].x2)/2){
                    // По часовой стрелке
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y2 + DELTA};
                } else {
                    next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y2 + DELTA};
                }
            }
        } else {
            if(objects[object->obj_i].x2 == object->in.x){
                next_path[0] = (Position){object->in.x - DELTA, object->in.y};
                next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
            } else {
                next_path[0] = (Position){object->in.x, object->in.y - DELTA};
                next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
            }
            next_path[2] = next_path[1];
        }

    } else if(sector == 1){

        if(get_type_direction(object, object->obj_i, sector) == 2){
            if(objects[object->obj_i].x2 == object->in.x){

                char dir;
                //Снизу вверх
                next_path[0] = (Position){object->in.x - DELTA, object->in.y};
                if(object->in.y <= objects[object->obj_i].y1 + (objects[object->obj_i].y1 + objects[object->obj_i].y2)/2){
                    // По часовой стрелке
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x1 + DELTA, next_path[1].y};
                } else {
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x1 + DELTA, next_path[1].y};
                }

            } else {

                char dir;
                //Справа налево
                next_path[0] = (Position){object->in.x, object->in.y - DELTA};
                if(object->in.x <= objects[object->obj_i].x1 + (objects[object->obj_i].x1 + objects[object->obj_i].x2)/2){
                    // По часовой стрелке
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y1 - DELTA};
                } else {
                    next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y1 - DELTA};
                }
            }
        } else {
            if(objects[object->obj_i].x2 == object->in.x){
                next_path[0] = (Position){object->in.x - DELTA, object->in.y};
                next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
            } else {
                next_path[0] = (Position){object->in.x, object->in.y + DELTA};
                next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
            }
            next_path[2] = next_path[1];
        }

    } else if(sector == 2){

        if(get_type_direction(object, object->obj_i, sector) == 2){
            if(objects[object->obj_i].x1 == object->in.x){

                char dir;
                //Сверху вниз
                next_path[0] = (Position){object->in.x + DELTA, object->in.y};
                if(object->in.y <= objects[object->obj_i].y1 + (objects[object->obj_i].y1 + objects[object->obj_i].y2)/2){
                    //
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x2 - DELTA, next_path[1].y};
                } else {
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x2 - DELTA, next_path[1].y};
                }

            } else {

                char dir;
                //Справа налево
                next_path[0] = (Position){object->in.x, object->in.y - DELTA};
                if(object->in.x <= objects[object->obj_i].x1 + (objects[object->obj_i].x1 + objects[object->obj_i].x2)/2){
                    // По часовой стрелке
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y1 - DELTA};
                } else {
                    next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y1 - DELTA};
                }
            }
        } else {
            if(objects[object->obj_i].x1 == object->in.x){
                next_path[0] = (Position){object->in.x + DELTA, object->in.y};
                next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
            } else {
                next_path[0] = (Position){object->in.x, object->in.y + DELTA};
                next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
            }
            next_path[2] = next_path[1];
        }

    } else if(sector == 3){

        if(get_type_direction(object, object->obj_i, sector) == 2){
            if(objects[object->obj_i].x1 == object->in.x){

                char dir;
                //Сверху вниз
                next_path[0] = (Position){object->in.x + DELTA, object->in.y};
                if(object->in.y <= objects[object->obj_i].y1 + (objects[object->obj_i].y1 + objects[object->obj_i].y2)/2){
                    //
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y2 + DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA})
                        && check_line((Position){next_path[0].x, objects[object->obj_i].y1 - DELTA}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y1 - DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y1 - DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x2 - DELTA, next_path[1].y};
                } else {
                    next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
                    next_path[2] = (Position){objects[object->obj_i].x2 - DELTA, next_path[1].y};
                }
                }
            } else {

                char dir;
                //Слева направо
                next_path[0] = (Position){object->in.x, object->in.y - DELTA};
                if(object->in.x <= objects[object->obj_i].x1 + (objects[object->obj_i].x1 + objects[object->obj_i].x2)/2){
                    // По часовой стрелке
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 1;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    }
                } else {
                    if(check_line(next_path[0], (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x2 - DELTA, next_path[0].y}, (Position){objects[object->obj_i].x2 - DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 2;
                    } else if(check_line(next_path[0], (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y})
                        && check_line((Position){objects[object->obj_i].x1 + DELTA, next_path[0].y}, (Position){objects[object->obj_i].x1 + DELTA, objects[object->obj_i].y2 + DELTA})){
                        dir = 1;
                    }
                }
                if(dir == 1){
                    next_path[1] = (Position){objects[object->obj_i].x1 + DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y2 + DELTA};
                } else {
                    next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
                    next_path[2] = (Position){next_path[1].x, objects[object->obj_i].y2 + DELTA};
                }
            }

        } else {
            if(objects[object->obj_i].x1 == object->in.x){
                next_path[0] = (Position){object->in.x + DELTA, object->in.y};
                next_path[1] = (Position){next_path[0].x, objects[object->obj_i].y2 + DELTA};
            } else {
                next_path[0] = (Position){object->in.x, object->in.y - DELTA};
                next_path[1] = (Position){objects[object->obj_i].x2 - DELTA, next_path[0].y};
            }
            next_path[2] = next_path[1];
        }


    //return next_path;
}

/*
* Возвращает 2, если двух-шаговый путь, 1 - если одношаговый.
*/
char PathFinder::get_type_direction(Cross *cross, char i, char sector){
    if(sector == 0 || sector == 1){
        if(cross->in.x == objects[i].x2 && cross->out.x == objects[i].x1)
            return 2;
        if(sector == 0 && cross->in.y == objects[i].y1 && cross->out.y == objects[i].y2)
            return 2;
        if(sector == 1 && cross->in.y == objects[i].y2 && cross->out.y == objects[i].y1)
            return 2;
    } else {
        if(cross->in.x == objects[i].x1 && cross->out.x == objects[i].x2)
            return 2;
        if(sector == 3 && cross->in.y == objects[i].y1 && cross->out.y == objects[i].y2)
            return 2;
        if(sector == 2 && cross->in.y == objects[i].y2 && cross->out.y == objects[i].y1)
            return 2;
    }
    return 1;
}

bool PathFinder::check_line(Position start, Position finish){
    for(char i=0; i<objects_count; ++i){
        char sector;
        if(finish.x - start.x == 0){
            if(finish.y - start.y > 0){
                sector = 4;
            } else {
                sector = 6;
            }
        } else if(finish.x - start.x > 0){
            sector = 5;
        } else {
            sector = 7;
        }
        Cross temp = check_one_side(i, sector, finish.x, finish.y, start.x, start.y);
        if(temp.in.x == -1 && temp.in.y == -1){
            return false;
        }
    }
    return true;
}

