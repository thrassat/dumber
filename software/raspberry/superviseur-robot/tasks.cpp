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

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches : 99 plus prio que 1
// Ordre théorique ok, a vérifier en pratique 
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 23
#define PRIORITY_TSENDTOMON 25
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20 // ELLE ETAIT A 20? 29.05
#define PRIORITY_TCAMERA 21
//priorité à changer peut être, on met une prio plus petite que mov car période plus grande
#define PRIORITY_TBATTERY 23
#define PRIORITY_TWD 22
#define PRIORITY_TIMAGEPOS 24
#define PRIORITY_TARENE 19





/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_watchdog, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_captureArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_position, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_cameraStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_errorCompteur, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_mutex_create(&mutex_savedArena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_arretConnexion, NULL)) {
        exit(EXIT_FAILURE);
    }

    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_mutex_create(&mutex_arenaAvailable, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       		// icount a 0					  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    // BATTERY 
    
       //Normalement pas utile car le sem get battery renvoi le niveau de batterie directement 
   /* if (err = rt_sem_create(&sem_batLevel, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }*/
    if (err = rt_sem_create(&sem_getbattery, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    // WATCHDOG
    
    if (err = rt_sem_create(&sem_reloadCpt, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
   // expiration? 
    
    // CAMERA 
    
    if (err = rt_sem_create(&sem_startCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    //la tâche pour la lecture de la batterie
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
   
    if (err = rt_task_create(&th_watchdog, "th_watchdog", 0, PRIORITY_TWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
<<<<<<< HEAD
   /* PRIORITE  if (err = rt_task_create(&th_image, "th_image", 0, PRIORITY_TBATTERY, 0)) {
=======
   if (err = rt_task_create(&th_imagePos, "th_imagePos", 0, PRIORITY_TIMAGEPOS, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_startCamera, "th_startCamera", 0, PRIORITY_TCAMERA, 0)) {
>>>>>>> master
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_arena, "th_arena", 0, PRIORITY_TARENE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    } */
    
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*500, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_imagePos, (void(*)(void*)) & Tasks::ImagePos, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_arena, (void(*)(void*)) & Tasks::ArenaTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startCamera, (void(*)(void*)) & Tasks::StartCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::GetBatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
     
    
    if (err = rt_task_start(&th_watchdog, (void(*)(void*)) & Tasks::WatchdogTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
   
    

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
   
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);

        monitor.Write(msg); // The message is deleted with the Write
        
        // Mecanism en cas de perte de connexion fonctionalité 9
        rt_mutex_acquire(&mutex_arretConnexion, TM_INFINITE);
        if(arretConnexion == true){
            Message *mess ;
            mess = new Message(MESSAGE_ROBOT_COM_CLOSE) ; 
            cout << " COMPTEUR DEPASSE , CO PERDUE, ROBOT ETEINT " << mess->ToString() << endl << flush ; 
            monitor.Write(mess) ;
        }
        rt_mutex_release(&mutex_arretConnexion);
        
       
        rt_mutex_release(&mutex_monitor);
        
        
        
       
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
 
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            cout << " Message MONITOR_LOST reçu : Perte de communication entre le superviseur et le moniteur " << endl << flush ;
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
            
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
            
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD) ) {
             rt_sem_v(&sem_startRobot);
             
             // MAJ de la var globale associée si démarrage avec WD
            rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
            watchdog = 1;
            rt_mutex_release(&mutex_watchdog);
             
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
            //Rajouté par Omar et Antoine : demande ouverture caméra
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            cout << "OUVERTURE CAM RECU" << endl << flush;
             //on incrémente le sémaphore, pas Broadcast car un seul thread le regarde
            rt_sem_v(&sem_startCamera);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            //TODO
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
            captureArena=ARENA_CAPTURE_REQUEST;
            rt_mutex_release(&mutex_captureArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            cout << "JE SUIS DANS LE IF CONFIRM" << endl << flush;
            rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
            captureArena=IS_VALIDATED;
            rt_mutex_release(&mutex_captureArena);
            cout << "J'ai changé captureArena" << endl << flush;
        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
            captureArena=IS_NOT_VALIDATED;
            rt_mutex_release(&mutex_captureArena);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
             rt_mutex_acquire(&mutex_position, TM_INFINITE);
             requetePosition = REQUETE_CALCUL_POSITION;
             rt_mutex_release(&mutex_position);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
             rt_mutex_acquire(&mutex_position, TM_INFINITE);
             requetePosition = PAS_DE_REQUETE_POSITION;
             rt_mutex_release(&mutex_position);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot. // START ROBOT 
 */
void Tasks::StartRobotTask(void *arg) {
    int wd ; 
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE); // decremente sem de 1 
        rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
        wd = watchdog ; 
        rt_mutex_release(&mutex_watchdog);
        
        if (wd == 0) {
            cout << "Start robot without watchdog ("<<endl << flush  ;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD()); 
            compteur_robot(msgSend) ; 
            rt_mutex_release(&mutex_robot);
           
        } else if (wd == 1) {
            cout << "Start robot with watchdog ("<<endl << flush;
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD()); 
            compteur_robot(msgSend) ; 
            rt_mutex_release(&mutex_robot);

        }
        // L'ack ou No ACK est envoyé par le thread opencomrobot
        // Indistinctement du démarrage avec wd ou non 
        else {
            cout << "ERREUR VARIABLE GLOBALE WATCHDOG (";
        } 
        cout << msgSend->GetID();
        cout << ")" << endl;
        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {    //Possible erreur : doit être avec compare ID !! !et pas ==
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}


/**
 * @brief Thread handling control of the robot. // MOVE TASK
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message *msgAnswer; 
     
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    //100 ms

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgAnswer = robot.Write(new Message((MessageID)cpMove));
            compteur_robot(msgAnswer) ; 
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread handling control of the robot. // BATTERY TASK 
 */

 void Tasks::GetBatteryTask(void *arg) {
    int rs;
    Message * level ;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    //on set la période à 500ms
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    

    while (1) {
        rt_task_wait_period(NULL);
        //cout << "Get battery" << endl << flush ; 
        
        //on y va si le robot a bien été started
        
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        if (rs == 1) {
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
           // Message level = robot.GetBattery(); //recupère le niveau de battery dans la variable level 
            level = robot.Write(robot.GetBattery()); 
            //robot.GetBattery renvoi le message pour avoir la batterie et as réponse et dans level
            compteur_robot(level) ; 
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToMon, level);
            cout << level->ToString() << endl << flush;
        }
        
    }
}


// AVANCEMENT : Algortithmiquement batterie a l'air ok, le vérifier / faire vérifier ------- Faire le thread th_watchdog puis après voir les thread normaux si y'a des trucs modifs t
 
 
 /**
 * @brief Thread handling control of the robot. // WATCHDOG TASK 
 */
 void Tasks::WatchdogTask(void *arg) {
     int rs,wd ; 
    Message * msgSend; 
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
   // set période d'1 sec 
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        cout << " (Re)lancement du thread Watchdog" << endl << flush ; 
        
        //On recupère les mutex Robot Started et watchdog 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        
        rt_mutex_acquire(&mutex_watchdog, TM_INFINITE);
        wd = watchdog;
        rt_mutex_release(&mutex_watchdog);
    
    
    
    
        if ((rs ==1) && (wd==1)) {  // envoi périodique du message de reload WD au robot 
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.ReloadWD()); 
            cout << " RELOAD WD : " << msgSend->ToString() << endl << flush ; 
            compteur_robot(msgSend) ; 
            rt_mutex_release(&mutex_robot);
            
        }  
        rt_task_wait_period(NULL);

    }
}
 
/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

void Tasks::compteur_robot(Message *msg){
    if(msg->CompareID(MESSAGE_ANSWER_ROBOT_ERROR)) {
       rt_mutex_acquire(&mutex_errorCompteur, TM_INFINITE);
       errorCompteur++;
       if(errorCompteur > 3){
           //mutex
           rt_mutex_acquire(&mutex_arretConnexion, TM_INFINITE);
           arretConnexion = true;
           rt_mutex_release(&mutex_arretConnexion);

       }
       rt_mutex_release(&mutex_errorCompteur);
    }
    else { 
       rt_mutex_acquire(&mutex_errorCompteur, TM_INFINITE);
       errorCompteur=0;
       rt_mutex_release(&mutex_errorCompteur);
    }  
}

/**
 * @brief Thread handling image sending and position computing and sending.
 */
void Tasks::ImagePos(void *arg) {
    //les trois variables ci dessous seront affectées avec les mutex
    int cs; //camera started
    int captArena;
    
    cout << "Start " << "image arene pos" << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task ImagePos starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    //100 ms
    
    Arena * sArena;
    Img * image;
    MessagePosition * msgPosition;
    Position position;
    
    bool arAv;
    
    
    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic image pos" << endl << flush;
        
        //on récupère la variable protégée par le mutex mutex_cameraStarted
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cs = cameraStarted;
        rt_mutex_release(&mutex_cameraStarted);
        
        if (cs == CAMERA_STARTED) {
          
           rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
           captArena=captureArena;
           rt_mutex_release(&mutex_captureArena);

            //on envoie l'image
            if (captArena == IS_NOT_CAPTURING_ARENA) {
                
                int reqPosition;
                /*****************Img part**********************/
                cout << "Calcul d'image demandé" << endl << flush;
                
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                cout << "mutex camera acquired" << endl << flush;
                bool isOpen = camera.IsOpen();
                if (isOpen) {
                    cout << "CAMERA OPEN!" << endl << flush;
                    image = camera.Grab().Copy();
                } else {
                    cout << "CAMERA CLOSED!" << endl << flush;
                }
                cout << "grabed" << endl << flush;
                rt_mutex_release(&mutex_camera);
                cout << "mutex camera released" << endl << flush;
                
               rt_mutex_acquire(&mutex_position, TM_INFINITE);
               reqPosition = requetePosition;
               rt_mutex_release(&mutex_position);
               /********************Img part***********************/ 
              
               /********************Position part******************/
               
                //on calcule la position si demandé
                if (reqPosition == REQUETE_CALCUL_POSITION) {
                    
                        rt_mutex_acquire(&mutex_arenaAvailable, TM_INFINITE);
                        arAv = arenaAvailable;
                        rt_mutex_release(&mutex_arenaAvailable);

                        if (arAv) {
                            
                            rt_mutex_acquire(&mutex_savedArena, TM_INFINITE);
                            sArena = savedArena;
                            rt_mutex_release(&mutex_savedArena);

                            cout << "Calcul de position demandé" << endl << flush;
                            std::list<Position> positions = image->SearchRobot(*sArena);
                            

                            if (positions.empty()) {
                                //le robot n'a pas été trouvé
                                position.NotFound();
                            } else {
                                position=positions.front();
                            }
                            image->DrawRobot(position);

                            
                            
                            //crée et envoie le message
                            msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, position);
                            WriteInQueue(&q_messageToMon, msgPosition);
                        } else {
                            cout << "Please define an arena before asking for position" << endl << flush;
                            position.NotFound();
                            msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, position);
                            WriteInQueue(&q_messageToMon, msgPosition);
                        }
                }
                /***********************end position part**************/   
                 //on convertit l'image en string pour le mettre dans le message
                MessageImg * msgImage = new MessageImg(MESSAGE_CAM_IMAGE, image);
                cout << "msg créé" << endl << flush;
                WriteInQueue(&q_messageToMon, msgImage); // msgSend will be deleted by sendToMon
                cout << "written in queue" << endl << flush;
           }
                

        }
        cout << endl << flush;
    }
}


/**
 * @brief Thread handling arena searching
 */
void Tasks::ArenaTask(void *arg) {
    cout << "Start " << " arena task" << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    
    Arena arena;
    int cs;
    
    Img * image;
    
    
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    /**************************************************************************************/
    /* The task ArenaTask starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        rt_task_wait_period(NULL);
        
      //on récupère la variable protégée par le mutex mutex_cameraStarted
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cs = cameraStarted;
        rt_mutex_release(&mutex_cameraStarted);
             
        if (cs == CAMERA_STARTED) {
            int captArena;

            rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
            captArena=captureArena;
            rt_mutex_release(&mutex_captureArena);
            
           // cout << "CAPT ARENA " << captArena << endl << flush;

            //on calcule l'arène
            if (captArena == ARENA_CAPTURE_REQUEST) {
                MessageImg * msgArena;
                
                cout << "Calcul d'arène demandé" << endl << flush;
                
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                image = camera.Grab().Copy();
                rt_mutex_release(&mutex_camera);                
                
                arena = image->SearchArena();
                image->DrawArena(arena);
                msgArena = new MessageImg(MESSAGE_CAM_IMAGE, image);
                WriteInQueue(&q_messageToMon, msgArena);
           
                rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
                captureArena=IS_WAITING_VALIDATION;
                rt_mutex_release(&mutex_captureArena);
            }
            cout << captArena << endl << flush;
            //si la recherche est validée
            if (captArena == IS_VALIDATED) {
                
                //on svg l'arène
                rt_mutex_acquire(&mutex_savedArena, TM_INFINITE);
                savedArena = &arena;
                rt_mutex_release(&mutex_savedArena);
                
                rt_mutex_acquire(&mutex_arenaAvailable, TM_INFINITE);
                arenaAvailable=true;
                rt_mutex_release(&mutex_arenaAvailable);
                
                cout << "ARENE VALIDEE" << endl << flush;
                
                /*
                 * on repasse la variable captureArena à IS_NOT_CAPTURING_ARENA
                 * pour repasser au monde d'envoie périodique
                 * */
                rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
                captureArena=IS_NOT_CAPTURING_ARENA;
                rt_mutex_release(&mutex_captureArena);
                
            }
            
            //si la recherche est annulée
            if (captArena == IS_NOT_VALIDATED) {
                /*
                 * on repasse la variable captureArena à IS_NOT_CAPTURING_ARENA
                 * pour repasser au monde d'envoie périodique
                 * */
                rt_mutex_acquire(&mutex_captureArena, TM_INFINITE);
                captureArena=IS_NOT_CAPTURING_ARENA;
                rt_mutex_release(&mutex_captureArena);
                
            }
            
        }
        cout << endl << flush;
    }
}







/**
 * @briefThread handling the camera start
 */
void Tasks::StartCamera(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    //on attend que receiveFromMon ait reçu le msg open_cam
    rt_sem_p(&sem_startCamera, TM_INFINITE);
    
    bool isOpen = false;
    
    
    cout << "Creation de la caméra" << endl << flush;
    
    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
    isOpen = camera.Open();
    rt_mutex_release(&mutex_camera);
     
     if(isOpen){
        rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted = CAMERA_STARTED;
        rt_mutex_release(&mutex_cameraStarted);  
   
        cout << "CAMERA PEUT ETRE START" << endl << flush;
     } else {
        Message *nack_camera = new Message(MESSAGE_ANSWER_NACK);
        WriteInQueue(&q_messageToMon,nack_camera);
     }
    
     cout << "Caméra créée" << endl << flush;
    
}








}
