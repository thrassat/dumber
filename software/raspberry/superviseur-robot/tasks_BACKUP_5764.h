/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TASKS_H__
#define __TASKS_H__

#define CAMERA_NOT_STARTED 0
#define CAMERA_STARTED 1

#define IS_NOT_CAPTURING_ARENA 0
#define ARENA_CAPTURE_REQUEST 1
#define IS_WAITING_VALIDATION 2
#define IS_VALIDATED 3
#define IS_NOT_VALIDATED 4


#define PAS_DE_REQUETE_POSITION 0
#define REQUETE_CALCUL_POSITION 1

#define ARENA_AVAILABLE 0
#define ARENA_NOT_AVAILABLE 1

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    int robotStarted = 0;
<<<<<<< HEAD
    int watchdog = 0 ; 
    int errorCompteur=0; //Compte le nombre d'erreur d'ecriture 
    bool arretConnexion = false;
=======
    
    //protégée par le mutex_arene
    int captureArena = IS_NOT_CAPTURING_ARENA;
    //protégée par le mutex_cameraStarted
    int cameraStarted = CAMERA_NOT_STARTED;
    //protégée par le mutex_position
    int requetePosition = PAS_DE_REQUETE_POSITION;
    // Acces à la caméra
    Camera camera;
    
>>>>>>> master
    int move = MESSAGE_ROBOT_STOP;
    
    bool arenaAvailable = ARENA_AVAILABLE;
    
    
    
    //pour sotcket les adresses svg par l'utilisateur
    Arena * savedArena;
   // int batteryLevel = BatteryLevel.BATTERY_UNKNOWN;
    
    
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;
    
    //threads gestion robot
    RT_TASK th_watchdog;
    RT_TASK th_battery;
    
        
    //threads caméras
    RT_TASK th_imagePos;
    RT_TASK th_arena;
    RT_TASK th_startCamera;
    
    
    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;

    //mutex gestion robot
    RT_MUTEX mutex_watchdog;
    RT_MUTEX mutex_errorCompteur;
    RT_MUTEX mutex_arretConnexion ;
    
    //mutex caméra
    RT_MUTEX mutex_captureArena;
    RT_MUTEX mutex_position;
    RT_MUTEX mutex_cameraStarted;
    RT_MUTEX mutex_savedArena;
    RT_MUTEX mutex_camera;
    RT_MUTEX mutex_arenaAvailable;
    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;
    
    //semaphore gestion_robot
    RT_SEM sem_getbattery ; 
    RT_SEM sem_expiration ; 
    RT_SEM sem_reloadCpt;
    //RT_SEM sem_batLevel; Supprime si on considere que get battery recupere directement la reponse du robot, supp du draw.io si on laisse commenté
    
    //semaphore caméra
    RT_SEM sem_startCamera;
   

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    // AJOUT DE TOUT LES MSG QUEUE NON DECLARE ? 
    
  
     
    
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
<<<<<<< HEAD
    // Ours 
    
    // Task thread
    
    void GetBatteryTask(void *arg) ; 
    void WatchdogTask(void *arg) ; 
     
     // Auxiliary task
    
    void compteur_robot(Message *msg) ; 
    
=======
    /**
     * @brief Thread handling image sending and position computing and sending.
     */
    void ImagePos(void *arg);
    
    /**
     * @brief Thread handling arena searching
     */
    void ArenaTask(void *arg);
    
    /**
     * @brief Thread handling the camera start
     */
    void StartCamera(void *arg);
>>>>>>> master
    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);
    
    /**
     * @brief
     */
    void ImageArenaPos(void *args);
};

#endif // __TASKS_H__ 

