#include <ps2.h>
#include <QueueList.h>
#include <PathFinder.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Ultra.h>
#include <Servo.h> 
#include "types.h"

#define bool char
#define true 1
#define false 0
#define PARROT(x) (float) x / 1000.0 * 3.4 * 2
#define MIN_X -1
#define MAX_X 100
#define MIN_Y -1
#define MAX_Y 200
#define DELTA 10
#define PLATFORM_RADIUS 5.8

Point active_position;
int active_degree; //0 - 359

Obstacle obstacles[20];
int count_obstacles = 0;

QueueList <Point> path_queue;

//commands
Command active_command;
QueueList <Command> commands;

//tasks
Point active_task;
QueueList <Point> tasks;

Servo left_servo;
Servo right_servo;

//flags
bool isAnswerReceived = false;
bool isStopped = false;
bool isNewCommandQueued = false;
bool isCorrectionQueued = false;

bool is_speed_of_light = false;
bool have_temp_obstacles = false;

bool canGo = false;
bool isAlreadyStarted = false;

//storing received answer
int distance0 = 0;
int distance1 = 0;
int receivedCommand = 0;

unsigned long prev_loop = 0;

//sensor timing
unsigned long sensorTime = 0;
unsigned long prevSensorTime = 0;
unsigned char ignored_sensor_count = 0;
unsigned int sound1 = 100;
unsigned int sound2 = 100;
unsigned int sound3 = 100;
unsigned int sound4 = 100;
unsigned int sound5 = 100;

//sensors
//Maxbotix ultraSoundSensor(A0, Maxbotix::AN, Maxbotix::LV);
int infraRedSensor = A2;
Ultra ultraSoundSensor(8,7);

bool isSensStop;
bool top_stop;
bool bottom_stop;

const int criticalDistance = 25;

//general timing
const long shutdownTime = 89000;
long startTime = 0;

//mouse
PS2 mouse(6, 5);
PathFinder pathfinder;
char mstat;
char mouseDeltaX;
char mouseDeltaY;
int mouseX = 0;
int mouseY = 0;
float mouseDistanceX = 0;
float mouseDistanceY = 0;


void debug_obs(int x1, int y1, int x2, int y2){
	Serial.print("Prestart obs");
	Serial.print(x1);
	Serial.print(" ");
	Serial.print(y1);
	Serial.print(" ");
	Serial.print(x2);
	Serial.print(" ");
	Serial.println(y2);
}

/* start_data
*  Тут должен был располагаться список объектов на поле
*/
void start_data(char direction){ //TODO
	obstacles[count_obstacles++] = (Obstacle){-1, 300, 0, 0};
	debug_obs(-1, 300, 0, 0);
	obstacles[count_obstacles++] = (Obstacle){0, 301, 200, 300};
	debug_obs(0, 301, 200, 300);
	obstacles[count_obstacles++] = (Obstacle){200, 300, 201, 0};
	debug_obs(200, 300, 201, 0);
	obstacles[count_obstacles++] = (Obstacle){0, 0, 200, -1};
	debug_obs(0, 0, 200, -1);
	if(direction){
		
	} else {
		obstacles[count_obstacles++] = (Obstacle){0, 115, 35, 35};
		debug_obs(0, 115, 35, 35);
		obstacles[count_obstacles++] = (Obstacle){0, 265, 35, 185};
		debug_obs(0, 265, 35, 185);
		obstacles[count_obstacles++] = (Obstacle){170, 30, 200, 0};
		debug_obs(170, 30, 200, 0);
		obstacles[count_obstacles++] = (Obstacle){170, 300, 200, 270};
		debug_obs(170, 300, 200, 270);
		obstacles[count_obstacles++] = (Obstacle){85, 170, 125, 130};
		debug_obs(85, 170, 125, 130);
	}
}

/* calculate_position
*  Собственно, изменяет текущую позицию робота.
*/
void calculate_position(unsigned char command, int X, int Y){
	if(command == 1){
		//Следовательно, робот ехал вперед или назад. Смещение по Y - попугаи.
		// Мы очень любим попугаев)
		active_position.x += Y * sin(active_degree);
		active_position.y += Y * cos(active_degree);
		//TODO учитывать еще смещение по X
	} else if(command == 2){
		//Ну значит мы поворачивали. Важным считать смещение по X - градусы.
		active_degree += X;

		//Поправим значения до уровня 0-359
		while(active_degree >= 360){
			active_degree -= 360;
		}
		while(active_degree < 0){
			active_degree += 360;
		}

		Serial.print("Set degree: ");
		Serial.print(X);
		Serial.print(" ");
		Serial.println(active_degree);

	} else if(command == 4){
		//От тут веселье, ибо мы летели со скоростью света! Соответственно - любые значения мыши не корректны)
		//Но мы юзаем одометрию
		//Поправка - нихрена мы реально не юзаем
		active_position.x += Y * sin(active_degree);
		active_position.y += Y * cos(active_degree);
	}
}

/* erase_commands
*  Очистка всех списков команд\задач.
*/
void erase_commands(){
	while(!commands.isEmpty()){
		commands.pop();
	}
	while(!path_queue.isEmpty()){
		path_queue.pop();
	}
}

/* task_to_command
*  перевод задач в команды. Я не помню, какая это версия: рабочая или нет.
*/
void task_to_command(){
	while(!path_queue.isEmpty()){
		//get_len_line(int *target_x, int *target_y, int *robot_x, int *robot_y)
		Point target;
		target = path_queue.pop();
		float x = 0 + sin(active_degree);
		float y = 10 + cos(active_degree);
		int target_x = target.x - active_position.x;
		int target_y = target.y - active_position.y;
		Serial.print(target_x);
		Serial.print("\t");
		Serial.println(target_y);
		float d;
		//d = acos((x*target_x + y*target_y)/(sqrt(x*x + y*y)*sqrt(target_x*target_x + target_y*target_y))) * 180 / PI;
		d = (atan2(target_y, target_x) * 180 / PI) - 90;
		//Serial.println(d);
		unsigned long r;
		//r = get_len_line(&target.x, &target.y, &active_position.x, &active_position.y); //TODO

		if(d!=0){
			commands.push((Command){2, d, 0});
		}
		if(r!=0){ 
			commands.push((Command){1, r, 0});
		}
	}
}

void setup() {
	Serial.begin(115200);
	Serial2.begin(115200);
	Serial.setTimeout(5);
	Serial2.setTimeout(5);
	pinMode(13, OUTPUT);

	//starting cord pin
	pinMode(2, INPUT_PULLUP);

	//field's side chooser button pin
	pinMode(9, INPUT_PULLUP);

	left_servo.attach(10);
	right_servo.attach(11);
	left_servo.write(0);
	right_servo.write(70);

	isSensStop = false;
	top_stop = true;
	bottom_stop = true;

	isAlreadyStarted = false;
	canGo = false;

	//mouse init
	/*mouse.write(0xff); // reset
	mouse.read(); // ack byte
	mouse.read(); // blank
	mouse.read(); // blank
	mouse.write(0xf0); // remote mode
	mouse.read(); // ack
	delay(4000); //mouse needs ~3.5 sec to initialize :(*/

	//filling command queue (!)backwards order!
	if (!digitalRead(9)) {
		//yellow side
		start_data(1);
		//commands.push((Command) {4, 140, 0});
		commands.push((Command) {4, 120, 0, 1});
		commands.push((Command) {2, -95, 0, 0});
		commands.push((Command) {1, 60, 1, 0});
		commands.push((Command) {1, -60, 1, 0});

		commands.push((Command) {2, -170, 0});
		commands.push((Command) {1, 10, 0});
		commands.push((Command) {1, -2, 0});
		commands.push((Command) {1, 5, 0});
		commands.push((Command) {1, -2, 0});
		commands.push((Command) {2, 10, 0});
		commands.push((Command) {1, -5, 0});


		//commands.push((Command) {0, 0, 0});
		tasks.push((Point){44, 37});
		active_task = tasks.pop();
		active_position = (Point){44, 17};
		active_degree = 0;
	} else {
		//red side
		start_data(0);
		//commands.push((Command) {4, 140, 0});
		commands.push((Command) {4, 120, 0, 2});
		commands.push((Command) {2, 95, 0, 0});
		commands.push((Command) {1, 60, 1, 0});
		commands.push((Command) {1, -60, 0, 0});
		//commands.push((Command) {0, 0, 0});

		commands.push((Command) {2, 170, 0});
		commands.push((Command) {1, 10, 0});
		commands.push((Command) {1, -2, 0});
		commands.push((Command) {1, 5, 0});
		commands.push((Command) {1, -2, 0});
		commands.push((Command) {2, 10, 0});
		commands.push((Command) {1, -5, 0});

		tasks.push((Point){44, 37});
		active_task = tasks.pop();
		active_position = (Point){44, 17};
		active_degree = 0;
	}
	is_speed_of_light = true;
	active_command = commands.pop();
	//DEBUG
	//active_command = commands.pop();
	Serial.print("Prestart start ");
	Serial.print(active_position.x); 
	Serial.print(" ");
	Serial.println(active_position.y);
	//Serial.println("start");
	//Serial.println("com\t| dist\t| mx\t| my\t| mrawx\t| mrawy\t|");
	/*while(Serial2.available() != 0){
		digitalWrite(13, LOW);
		Serial2.parseInt();
		Serial2.parseInt();
		Serial2.parseInt();
	}*/
					
}


/* loop
*  тут я как понаписал лапшей в Москве, так тут так и осталось...
*/
void loop() {

	sensorTime = millis();

    //starting cord
	//Serial.println(!isAlreadyStarted);
	if (!digitalRead(2) && !isAlreadyStarted) {
		canGo = true;
		startTime = millis();
		isAlreadyStarted = true;
                Serial2.println(String(active_command.command) + String(" ") + String(active_command.distance));
	}

	if (canGo) {
		if (millis() - startTime > shutdownTime) {
			canGo = false;
			Serial2.println("3 0");
			Serial.println("Timer Stop!");
		} else if(sensorTime - prev_loop > 20){
			if(active_command.command != 0){
			
				if (Serial2.available() > 0) {
					digitalWrite(13, LOW);
					long tempReceivedCommand = Serial2.parseInt();
					if (tempReceivedCommand != 0) {
						receivedCommand = tempReceivedCommand;
					}
					long tempDistance0 = Serial2.parseInt();
					if (tempDistance0 != 0) {
						distance0 = tempDistance0;
					}
					long tempDistance1 = Serial2.parseInt();
					if (tempDistance1 != 0) {
						distance1 = tempDistance1;
					}
					if (tempDistance0 != 0 || tempDistance1 != 0) {
						isAnswerReceived = true;
					}

				}

				//TODO смещения
				/*if(active_command.not_sens == 0 && (active_command.command == 1 || active_command.command == 4) && abs(mouseDeltaX) > 1){
					is_speed_of_light = false;
					Serial2.println("3 0");
					calculate_position(active_command.command, mouseDistanceX, mouseDistanceY);
					calculate_position(2, mouseDistanceX, mouseDistanceY);
					if(mouseDistanceX > 0){
						//TODO
					} else {
						//TODO
					}
					while(Serial2.available() == 0){
					}
					digitalWrite(13, LOW);
					Serial2.parseInt();
					Serial2.parseInt();
					Serial2.parseInt();
					//
					Serial2.println("1 -5");
					while(Serial2.available() == 0){
					}
					digitalWrite(13, LOW);
					Serial2.parseInt();
					Serial2.parseInt();
					Serial2.parseInt();
					active_command.command = 1;
					active_command.distance = -5;
					isAnswerReceived = true; //Хитровыпендреж
				}*/

				//TODO Объект перед нами
				if (sensorTime - prevSensorTime > 20) {
					unsigned int inrfa = analogRead(infraRedSensor);
					//Serial.print("Inrfa: ");
					//Serial.println(inrfa);
					unsigned int sound = ultraSoundSensor.Range();
					if(sound != 0){
						sound5 = sound4;
						sound4 = sound3;
						sound3 = sound2;
						sound2 = sound1;
						sound1 = sound;
					}
					/*unsigned int */sound = (sound1 + sound2 + sound3 + sound4 + sound5) / 5;
					//Serial.print("Sound: ");
					//Serial.println(sound);
					top_stop = false;
					bottom_stop = false;
					if (inrfa > 470 && ignored_sensor_count == 0 && (active_command.command!=1 || active_command.distance > 0)) {
						if(!active_command.not_sens){
							is_speed_of_light = false;
							Serial2.println("3 0");
							Serial.println("Warning on top");
							top_stop = true;
							isSensStop = true;
							/*if(active_command.command == 4){
								calculate_position(active_command.command, mouseDistanceX, (distance0+distance1)/2);
							} else {
								calculate_position(active_command.command, mouseDistanceX, mouseDistanceY);
							}*/
							mouseX = 0;
                                        		mouseY = 0;
							//TODO Создать препятствие
							/*Point temp;
							temp.x = active_position.x + 0 + sin(active_degree);
							temp.y = active_position.y + 20 + cos(active_degree);
							obstacles[count_obstacles++] = (Obstacle){temp.x-5, temp.y+5, temp.x+5, temp.y-5};
							have_temp_obstacles = true;
							//Next
							calculate_position(active_command.command, mouseDistanceX, mouseDistanceY);*/
							active_command.command = 0;
							isAnswerReceived = false;
							ignored_sensor_count = 10;
						}
					}
					if(sound < 20 && (active_command.command!=1 || active_command.distance < 0)){
						if(!active_command.not_sens){
							is_speed_of_light = false;
							Serial2.println("3 0");
							Serial.println("Warning on bottom");
							bottom_stop = true;
							isSensStop = true;							
							active_command.command = 0;
							isAnswerReceived = false;
							//calculate_position(active_command.command, mouseDistanceX, mouseDistanceY);
							mouseX = 0;
                                        		mouseY = 0;
							//TODO Создать препятствие
							/*Point temp;
							temp.x = active_position.x + 0 + sin(active_degree);
							temp.y = active_position.y - 20 + cos(active_degree);
							obstacles[count_obstacles++] = (Obstacle){temp.x-5, temp.y+5, temp.x+5, temp.y-5};
							have_temp_obstacles = true;*/
						}
						//isStopped = false;
					}
					if(isSensStop && !top_stop && !bottom_stop){
						isSensStop = false;
					}
					prevSensorTime = sensorTime;
				}

				if(isAnswerReceived && active_command.command != 0){
					isAnswerReceived = false;
					Serial.println("Answer");
					Serial.print("receivedCommand: ");
					Serial.println(receivedCommand);
					/*Serial.print("Active distance ");
					Serial.println(active_command.distance);*/
					/*if(active_command.command == 4){
						calculate_position(active_command.command, mouseDistanceX, (distance0+distance1)/2);
					} else {
						calculate_position(active_command.command, mouseDistanceX, mouseDistanceY);
					}*/
					if(active_command.command == 1){
						Serial.println("Command 1");
						if(abs(active_task.x - active_position.x) < 2 && abs(active_task.y - active_position.y) < 2){
							//Задача выполнена!
							//TODO Выбрать следующую задачу
							if(!tasks.isEmpty()){
								active_task = tasks.pop();
							} else {
								active_task.x = -1;
								active_task.y = -1;
							}
							Serial.println("Task complete!");
							is_speed_of_light = false;
							erase_commands();
						} else {
							/*if(!is_speed_of_light && abs(mouseDistanceY - active_command.distance) > 0.5){
								Serial.println("You loser!");
								Serial.print(mouseDistanceY);
								Serial.print("\t");
								Serial.println(active_command.distance);
								erase_commands();
								//active_command.command = 0;
							} else {*/
								Serial.println("You Sniper!");
								if(!commands.isEmpty()){
									active_command = commands.pop();
									Serial.print("Set Command: ");
									Serial.print(active_command.command);
									Serial.print(" ");
									Serial.print(active_command.distance);
									Serial.print(" ");
									Serial.println(active_command.not_sens);
									Serial2.println(String(active_command.command) + String(" ") + String(active_command.distance));
								} else {
									/*if(is_speed_of_light){
										is_speed_of_light = false;
										if()
									}*/
									
									//Че за на? Пересчитать!
									//active_command.command = 0;
								}
							//}
						}
					} else if(active_command.command == 2 || active_command.command == 4){
						Serial.println("Command 2|4");
						active_command = commands.pop();
						Serial2.println(String(active_command.command) + String(" ") + String(active_command.distance));
					}
					mouseX = 0;
                                        mouseY = 0;
				}
				Serial.print("Position: ");
				Serial.print(active_position.x);
				Serial.print(" ");
				Serial.println(active_position.y);
			} else {
				//Serial.println("Other");
				if(active_task.x == -1 && active_task.y == -1){
					//Weekend!
				} else {
					if(isSensStop){
						/*if (Serial2.available() > 0) {
							digitalWrite(13, LOW);
							long tempReceivedCommand = Serial2.parseInt();
							if (tempReceivedCommand != 0) {
								receivedCommand = tempReceivedCommand;
							}
							long tempDistance0 = Serial2.parseInt();
							if (tempDistance0 != 0) {
								distance0 = tempDistance0;
							}
							long tempDistance1 = Serial2.parseInt();
							if (tempDistance1 != 0) {
								distance1 = tempDistance1;
							}
							if (tempDistance0 != 0 || tempDistance1 != 0) {
								isAnswerReceived = true;
							}
							if(isAnswerReceived){
								Serial.print("StopCommand: ");
								Serial.print(receivedCommand);
								Serial.print(" with: ");
								Serial.print(distance0);
								Serial.print(" ");
								Serial.println(distance1);
							}
				}*/
					}
					//update_path(active_position, active_task);
					//Serial.print("Active task:\t");
					//Serial.print(active_task.x);
					//Serial.print("\t");
					//Serial.println(active_task.y);
					//while(!path_queue.isEmpty()){
					//	Point temp = path_queue.pop();
					//	Serial.print(temp.x);
					//	Serial.print("\t");
					//	Serial.println(temp.y);
					//}
					//Serial.println("End of path!");
					//task_to_command();
					//if(have_temp_obstacles){
					//	--count_obstacles;
					//}
					//active_command = commands.pop();
					//Serial2.println(String(active_command.command) + String(" ") + String(active_command.distance));
					/*while(!commands.isEmpty()){
						Command temp = commands.pop();
						Serial.print(temp.command);
						Serial.print("\t");
						Serial.println(temp.distance);
					}*/
					//Serial.println("End of commands!");
					//active_command = commands.pop();
					//Serial2.println(String(active_command.command) + String(" ") + String(active_command.distance));
				}
			}
			prev_loop = sensorTime;
		}
		//Serial.println("End of cycle");
	} //end of starting cord if
}
